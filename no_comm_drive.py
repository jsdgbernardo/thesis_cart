#!/usr/bin/env python3
"""Open-loop baseline: publish fixed body-forward commands, with no cart-state subscription."""
import argparse
import signal
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class OpenLoopDriver(Node):
    def __init__(self, num_carts: int, speed: float):
        super().__init__('obj4_no_comm_driver')
        self.speed = speed
        self.pubs = [
            self.create_publisher(Twist, f'/cart_{i}/cmd_vel', 10)
            for i in range(1, num_carts + 1)
        ]
        self.timer = self.create_timer(0.1, self.publish_forward)

    def publish_forward(self):
        msg = Twist()
        msg.linear.x = self.speed
        for publisher in self.pubs:
            publisher.publish(msg)

    def stop(self):
        msg = Twist()  # Publish zero velocity for each cart before exit.
        for _ in range(4):
            for publisher in self.pubs:
                publisher.publish(msg)
            time.sleep(0.05)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--carts', type=int, choices=(2, 3), required=True)
    parser.add_argument('--speed', type=float, default=0.5)
    args = parser.parse_args()

    rclpy.init()
    node = OpenLoopDriver(args.carts, args.speed)
    running = True

    def request_stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
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
