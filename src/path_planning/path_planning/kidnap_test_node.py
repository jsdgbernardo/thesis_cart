#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import math
import time
import logging

class KidnappedRobotTimer(Node):
    def __init__(self):
        super().__init__('kidnapped_robot_timer')

        # Parameters
        self.true_x = 1.020496129989624  # set your known ground-truth x
        self.true_y = 0.04423929750919342 # set your known ground-truth y
        self.threshold = 0.2  # meters
        self.consecutive_required = 5  # number of consecutive readings below threshold

        self.counter = 0
        self.start_time = None

        # Subscribe to AMCL pose
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        # Create client for global localization
        self.cli = self.create_client(Empty, '/reinitialize_global_lcalization')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /reinitialize_global_localization service...')

        # Call the service
        self.get_logger().info('Calling global localization service...')
        req = Empty.Request()
        self.cli.call_async(req)

        # Start the timer
        self.start_time = time.time()
        self.get_logger().info('Timer started.')

    def amcl_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Calculate Euclidean distance to true position
        error = math.sqrt((x - self.true_x)**2 + (y - self.true_y)**2)

        # Check if within threshold
        if error < self.threshold:
            self.counter += 1
        else:
            self.counter = 0  # reset counter if outside threshold

        if self.counter >= self.consecutive_required:
            elapsed = time.time() - self.start_time
            self.get_logger().info(f'Converged! Error within {self.threshold} m for {self.consecutive_required} readings.')
            self.get_logger().info(f'Convergence time: {elapsed:.2f} seconds')
            logging.info(f'Converged! Error within {self.threshold} m for {self.consecutive_required} readings. Convergence time: {elapsed} seconds')
            rclpy.shutdown()  # stop node after convergence

def main(args=None):

    logging.basicConfig(
        filename="kidnaptest.txt",
        level=logging.INFO,
        format="[%(asctime)s] %(message)s"
    )

    rclpy.init(args=args)
    node = KidnappedRobotTimer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()