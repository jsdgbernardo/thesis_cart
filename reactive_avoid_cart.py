#!/usr/bin/env python3
"""Objective 4: independent cart avoidance WITHOUT inter-cart communication.

Each controller can ONLY sense nearby cart geometry/velocity through Gazebo's
/model_states as an IDEALIZED local proximity-sensor proxy (no cart IDs,
priority, transmitted intent, or /cart_states subscription). The logic makes
attempts at TURN_OUT -> PASS -> RETURN, and applies independent local holds.

This controller is NOT a full 2D LiDAR implementation, not a replica of the
proposed inter-cart maneuvering code, and it cannot guarantee deadlock-free
passing. Do not claim collision-free results without analyzing the ROS bags.
"""
import argparse
import math
import signal
import time
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node


@dataclass
class Body:
    x: float
    y: float
    yaw: float
    vx: float
    vy: float


def angle_wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v: float, low: float, high: float) -> float:
    return max(low, min(high, v))


class LocalController:
    def __init__(self, speed: float, safety_radius: float, protected: float,
                 aisle_width: float, world_half: float, sensor_range: float,
                 horizon: float):
        self.speed = speed
        self.safety = safety_radius
        self.protected = protected
        self.aisle_width = aisle_width
        self.world_half = world_half
        self.sensor_range = sensor_range
        self.horizon = horizon
        self.base = None
        self.mode = 'CLEAR'
        self.side = -1
        self.lane_offset = min(0.83, aisle_width / 2.0 - 0.28)
        self.pass_until = 0.0
        self.arm_after = -1.0
        self.last_reason = ''
        self.last_observation = float('inf')

    def local_coordinates(self, me: Body, p: Tuple[float, float]):
        base = self.base
        dx, dy = p[0] - base.x, p[1] - base.y
        c, s = math.cos(base.yaw), math.sin(base.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def world_coordinates(self, along: float, side: float):
        base = self.base
        c, s = math.cos(base.yaw), math.sin(base.yaw)
        return base.x + along * c - side * s, base.y + along * s + side * c

    def is_on_floor(self, x: float, y: float):
        # Corridors are a +/- aisle_width/2 cross in the Gazebo test world.
        # Reserve robot radius (rectangle half-diagonal=0.233m) + small margin.
        clearance = self.aisle_width / 2.0 - 0.28
        end = self.world_half - 0.28
        return (abs(x) <= end and abs(y) <= clearance) or (abs(y) <= end and abs(x) <= clearance)

    def perceived(self, me: Body, bodies: List[Body]):
        return [b for b in bodies if math.hypot(b.x - me.x, b.y - me.y) <= self.sensor_range]

    def closest_approach(self, me: Body, ob: Body):
        dx, dy = ob.x - me.x, ob.y - me.y
        # Use observed Gazebo velocities as idealized perceived motion, not
        # any neighbor state/intent topic. Cart motion is world-frame.
        svx, svy = me.vx, me.vy
        if abs(svx) + abs(svy) < 0.04 and self.mode == 'CLEAR':
            svx = self.speed * math.cos(me.yaw)
            svy = self.speed * math.sin(me.yaw)
        dvx, dvy = ob.vx - svx, ob.vy - svy
        rel_v2 = dvx * dvx + dvy * dvy
        t = clamp(-(dx * dvx + dy * dvy) / rel_v2, 0.0, self.horizon) if rel_v2 > 1e-5 else 0.0
        pred = math.hypot(dx + dvx * t, dy + dvy * t)
        return math.hypot(dx, dy), pred, t

    def closest(self, me: Body, obstacles: List[Body]):
        return min((math.hypot(ob.x - me.x, ob.y - me.y) for ob in obstacles), default=float('inf'))

    def needs_avoidance(self, me: Body, obstacles: List[Body]):
        here_s, here_d = self.local_coordinates(me, (me.x, me.y))
        for ob in obstacles:
            distance, prediction, approach_t = self.closest_approach(me, ob)
            ob_s, ob_d = self.local_coordinates(me, (ob.x, ob.y))
            ahead = ob_s > here_s - 0.3
            same_corridor = abs(ob_d - here_d) <= max(0.65, self.safety * 0.6)
            # Forecasted overlap; also allow immediate obstacles to trigger.
            if (distance < 2.3 and ahead and same_corridor) or (prediction < 0.83 and approach_t < self.horizon and ahead):
                return True
        return False

    def choose_side(self, me: Body, obstacles: List[Body]):
        along, lateral = self.local_coordinates(me, (me.x, me.y))
        # Evaluate both sides from purely perceived geometry. No explicit
        # cart identity, right of way, or negotiation is involved.
        scores = {}
        for side in (-1, +1):
            target_d = side * self.lane_offset
            samples = [self.world_coordinates(along + ahead, target_d)
                       for ahead in (0.7, 1.4, 2.1)]
            if not all(self.is_on_floor(x, y) for x, y in samples):
                scores[side] = -1e6
                continue
            score = 0.0
            for k, (x, y) in enumerate(samples):
                tt = (k + 1) * 0.9
                for ob in obstacles:
                    # Weakly predict neighbors' locations without asking them
                    # to stop or reveal intended maneuvers.
                    xx = ob.x + ob.vx * tt
                    yy = ob.y + ob.vy * tt
                    d = math.hypot(xx - x, yy - y)
                    score += min(d, 2.0) - 2.0 * max(0.0, 0.85 - d)
            # Simple independent right-side tendency breaks perfectly
            # symmetric head-on geometry; NOT an ID/priority ranking.
            if side == -1:
                score += 0.30
            scores[side] = score
        return max(scores, key=scores.get)

    def steer_to(self, me: Body, waypoint: Tuple[float, float], max_speed: float):
        dx, dy = waypoint[0] - me.x, waypoint[1] - me.y
        err = angle_wrap(math.atan2(dy, dx) - me.yaw)
        cmd = Twist()
        cmd.angular.z = clamp(2.5 * err, -1.25, 1.25)
        if abs(err) < 1.1:
            cmd.linear.x = max_speed * max(0.24, 1.0 - abs(err) / 1.1)
        return cmd

    def step(self, me: Body, observed: List[Body]):
        if self.base is None:
            self.base = Body(me.x, me.y, me.yaw, 0.0, 0.0)
        obstacles = self.perceived(me, observed)
        along, lateral = self.local_coordinates(me, (me.x, me.y))
        near = self.closest(me, obstacles)
        self.last_observation = near
        cmd = Twist()
        reason = ''

        if self.mode == 'CLEAR' and along >= self.arm_after and self.needs_avoidance(me, obstacles):
            side = self.choose_side(me, obstacles)
            if self.lane_offset <= 0.30 or not self.is_on_floor(*self.world_coordinates(along + 1.0, side * self.lane_offset)):
                self.mode = 'HOLD'
                reason = 'No geometrically feasible passing offset'
            else:
                self.side = side
                self.mode = 'TURN_OUT'
                obstacle_along = [self.local_coordinates(me, (o.x, o.y))[0] for o in obstacles]
                self.pass_until = max(along + 2.4, max(obstacle_along, default=along) + 1.0)
                reason = f'Independent bypass side={side:+d}, offset={self.lane_offset:.2f}m'

        if self.mode == 'CLEAR':
            cmd = self.steer_to(me, self.world_coordinates(along + 1.7, 0.0), self.speed)
        elif self.mode == 'TURN_OUT':
            cmd = self.steer_to(me, self.world_coordinates(along + 0.95, self.side * self.lane_offset), self.speed * 0.78)
            if abs(lateral - self.side * self.lane_offset) <= 0.13:
                self.mode = 'PASS'
                reason = 'Reached passing lane'
        elif self.mode == 'PASS':
            cmd = self.steer_to(me, self.world_coordinates(along + 1.6, self.side * self.lane_offset), self.speed)
            # Only return once ahead of conflict and centerline return seems
            # geometrically safe from currently sensed obstacles.
            if along >= self.pass_until:
                route = [self.world_coordinates(along + v, self.side * self.lane_offset * (1.0 - v / 1.6))
                         for v in (0.4, 0.8, 1.2, 1.6)]
                clear = all(self.is_on_floor(x, y) and
                            all(math.hypot(x - ob.x, y - ob.y) >= self.safety for ob in obstacles)
                            for x, y in route)
                if clear:
                    self.mode = 'RETURN'
                    reason = 'Passed obstacle; rejoining centerline'
        elif self.mode == 'RETURN':
            cmd = self.steer_to(me, self.world_coordinates(along + 1.5, 0.0), self.speed * 0.8)
            if abs(lateral) < 0.12 and abs(angle_wrap(me.yaw - self.base.yaw)) < 0.19:
                self.mode = 'CLEAR'
                self.arm_after = along + 1.0
                reason = 'Original lane recovered'
        elif self.mode == 'HOLD':
            # Retry if the sensed obstacle has moved or a side becomes free.
            if near > self.safety + 0.3:
                self.mode = 'CLEAR'
                self.arm_after = along - 0.2
                reason = 'Local geometry cleared; resuming'

        # Immediate protective stop (also during active bypass). Stops are
        # genuine failures if they persist, rather than forced successes.
        if near < self.protected + 0.04:
            cmd = Twist()
            reason = f'Protective HOLD: cart distance={near:.2f}m'
        # Look-ahead wall boundary: avoid driving the front of the footprint
        # into the aisle walls. Turning in place is still permitted.
        elif cmd.linear.x > 0:
            hx = me.x + 0.39 * cmd.linear.x * math.cos(me.yaw)
            hy = me.y + 0.39 * cmd.linear.x * math.sin(me.yaw)
            if not self.is_on_floor(hx, hy):
                cmd.linear.x = 0.0
                reason = 'Boundary hold: projected motion leaves aisle'
        self.last_reason = reason
        return cmd, self.mode, near, reason


class IndependentAvoidance(Node):
    def __init__(self, args):
        super().__init__(f'reactive_avoid_cart_{args.cart_id}')
        self.name = f'cart_{args.cart_id}'
        self.num = args.carts
        self.controller = LocalController(args.speed, args.stop_radius,
                                          args.protected_separation,
                                          args.aisle_width, args.world_half_width,
                                          args.sensor_range, args.prediction_horizon)
        self.pub = self.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)
        self.sub = self.create_subscription(ModelStates, '/gazebo/model_states', self.models, 10)
        self.my_body = None
        self.sensed = []
        self.last_frame = 0.0
        self.last_mode = ''
        self.last_hold = ''
        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info(
            f'{self.name}: local reactive TURN_OUT/PASS/RETURN, '
            f'no inter-cart messages, aisle={args.aisle_width:.1f} m')

    @staticmethod
    def body(pose, tw):
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                         1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        return Body(pose.position.x, pose.position.y, yaw,
                    tw.linear.x, tw.linear.y)

    def models(self, msg):
        bodies = []
        me = None
        for name, pose, tw in zip(msg.name, msg.pose, msg.twist):
            if name == self.name:
                me = self.body(pose, tw)
            elif name.startswith('cart_'):
                bodies.append(self.body(pose, tw))
        self.my_body = me
        self.sensed = bodies
        self.last_frame = time.monotonic()

    def update(self):
        cmd = Twist()
        if self.my_body is None or time.monotonic() - self.last_frame > 0.5:
            mode = 'SENSOR_HOLD'
            near, reason = float('inf'), 'Missing/stale Gazebo proximity proxy'
        else:
            cmd, mode, near, reason = self.controller.step(self.my_body, self.sensed)
        self.pub.publish(cmd)
        if mode != self.last_mode:
            self.get_logger().info(f'{self.name}: {mode} nearest={near:.2f}m {reason}')
            self.last_mode = mode
        if ('HOLD' in reason or 'hold' in reason) and reason != self.last_hold:
            self.get_logger().warn(f'{self.name}: {reason}')
            self.last_hold = reason
        if not ('HOLD' in reason or 'hold' in reason):
            self.last_hold = ''

    def stop(self):
        for _ in range(4):
            self.pub.publish(Twist())
            time.sleep(0.05)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cart-id', type=int, required=True)
    p.add_argument('--carts', type=int, choices=(2, 3), required=True)
    p.add_argument('--speed', type=float, default=0.5)
    p.add_argument('--stop-radius', type=float, default=1.2)
    p.add_argument('--protected-separation', type=float, default=0.616)
    p.add_argument('--prediction-horizon', type=float, default=3.0)
    p.add_argument('--sensor-range', type=float, default=3.3)
    p.add_argument('--aisle-width', type=float, default=2.8)
    p.add_argument('--world-half-width', type=float, default=12.0)
    args = p.parse_args()
    if not (1 <= args.cart_id <= args.carts):
        p.error('Invalid --cart-id')
    rclpy.init()
    node = IndependentAvoidance(args)
    running = True

    def shutdown(_sig, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
