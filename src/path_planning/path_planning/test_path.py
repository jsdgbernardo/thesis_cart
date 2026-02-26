import argparse
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return (0.0, 0.0, qz, qw)


class TestPath(Node):
    def __init__(self, service_name: str = 'compute_path_to_pose'):
        super().__init__('test_node')
        self.get_logger().info('Test node has been started.')
        self._action_client = ActionClient(self, ComputePathToPose, service_name)

    def make_pose_stamped(self, x: float, y: float, yaw: float, frame_id: str = 'map') -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        q = yaw_to_quaternion(float(yaw))
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        return ps

    def compute_path(self, start: PoseStamped, goal: PoseStamped, timeout_sec: float = 10.0):
        if not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('Action server not available: ComputePathToPose')
            return None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = ''
        goal_msg.use_start = True

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=timeout_sec)
        if not send_goal_future.done() or send_goal_future.result() is None:
            self.get_logger().error('Failed to send goal')
            return None

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done() or result_future.result() is None:
            self.get_logger().error('Get result failed or timed out')
            return None

        result = result_future.result().result
        return result.path


def main(args=None):
    parser = argparse.ArgumentParser(description='Compute shortest path via Nav2 ComputePathToPose')
    parser.add_argument('--start', nargs=3, type=float, metavar=('X','Y','YAW'),
                        default=[0.0, 0.0, 0.0], help='start pose x y yaw (radians)')
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X','Y','YAW'),
                        default=[1.0, 0.0, 0.0], help='goal pose x y yaw (radians)')
    parser.add_argument('--service', type=str, default='compute_path_to_pose', help='ComputePathToPose service name')
    parsed, unknown = parser.parse_known_args(args)

    rclpy.init(args=None)
    node = TestPath(service_name=parsed.service)

    start_ps = node.make_pose_stamped(*parsed.start)
    goal_ps = node.make_pose_stamped(*parsed.goal)

    path = node.compute_path(start_ps, goal_ps)
    if path is None:
        node.get_logger().error('No path returned')
    else:
        node.get_logger().info(f'Path received with {len(path.poses)} poses')
        for i, p in enumerate(path.poses):
            x = p.pose.position.x
            y = p.pose.position.y
            node.get_logger().info(f'  [{i}] x={x:.3f} y={y:.3f}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()