#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
  
    def __init__(self):
      super().__init__('image_publisher')

      # create publisher
      self.publisher_ = self.create_publisher(Image, 'camera/image', 10)

      timer_period = 0.1 # seconds
      self.timer = self.create_timer(timer_period, self.timer_callback)

      self.cap = cv2.VideoCapture(0) # webcam

      self.br = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
            self.get_logger().info('Publishing video frame')
        else:
            self.get_logger().error('Failed to capture video frame')
            
def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
