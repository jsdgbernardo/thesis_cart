import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np
import rclpy
from rclpy.node import Node

class CameraDetector(Node):

    def __init__(self, stream=0):
        super().__init__('camera_detector')

        # publisher for CompressedImage
        self.publisher = self.create_publisher(CompressedImage, '/camera/image/compressed', 10)
        
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                raise RuntimeError(f'Failed to open video stream: {self.stream}')
        except Exception as e:
            raise RuntimeError(f"Camera initialization failed: {e}")
        
        self.timer = self.create_timer(0.25, self.publish_frame) # 4 fps

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame from camera.")
            return 

        # compress frame as jpeg
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ret:
            self.get_logger().warn('Failed to compress frame')
            return

        msg = CompressedImage()
        msg.format = 'jpeg'
        msg.data = np.array(buffer).tobytes()
        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()