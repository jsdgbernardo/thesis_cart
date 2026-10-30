#!/usr/bin/env python3
"""Timed low-speed forward request for a supervised CA-1/2/3 demonstration.
Publishes via /cmd_vel_nav; obstacle controller can override/stop motion.
Run only with accessible physical motor power cutoff and adequate space.
"""
import argparse
import time
import rclpy
from geometry_msgs.msg import Twist


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=90.0)
    parser.add_argument('--speed', type=float, default=0.07)
    args = parser.parse_args()
    if not (0 < args.speed <= 0.07) or args.seconds < 0:
      parser.error('speed must be >0 and <=0.07; seconds must be >=0')

    rclpy.init()
    node = rclpy.create_node('ca_minimum_demo_sender')
    pub = node.create_publisher(Twist, '/cmd_vel_nav', 10)
    stop = Twist()
    try:
        print('Waiting for /cmd_vel_nav subscriber...')
        while pub.get_subscription_count() < 1:
            time.sleep(0.1)
        print('Ready; check /cart_state and the physical power cutoff.')
        input('Press ENTER to send the forward request. Ctrl+C cancels. ')
        cmd = Twist()
        cmd.linear.x = args.speed
        start = time.monotonic()
        while args.seconds == 0 or time.monotonic() - start < args.seconds:
            pub.publish(cmd)
            time.sleep(0.1)
    except (KeyboardInterrupt, EOFError):
        print('\nStopping test')
    finally:
        for _ in range(10):
            pub.publish(stop)
            time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()
        print('Zero command published. Check wheels are stopped.')

if __name__ == '__main__':
    main()
