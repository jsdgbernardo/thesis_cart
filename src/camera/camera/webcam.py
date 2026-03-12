#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
  
    def __init__(self):
      super().__init__('image_publisher')

      self.publisher_ = self.create_publisher(Image, 'camera/image', 10)

      self.subscription = self.create_subscription(
        Bool,
        'camera/capture',
        self.capture_callback,
        10
    )

      self.cap = cv2.VideoCapture(0) # webcam
      self.br = CvBridge()

      self.get_logger().info("Camera ready. Waiting for trigger...")

    def capture_callback(self, msg):
        if msg.data: 
            ret, frame = self.cap.read()

            if ret:
                self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
                self.get_logger().info("Published ONE frame")
            else:
                self.get_logger().error("Failed to capture frame")
            
def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()