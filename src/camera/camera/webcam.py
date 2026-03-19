#!/usr/bin/env python3

# Overall Flow:
# background thread -> drain camera -> latest frame
#       trigger -> capture_callback -> latest frame -> camera/image

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import threading

class Webcam(Node):

    def __init__(self):
        super().__init__('webcam')

        # topic publishers 
        self.publisher_ = self.create_publisher(Image, 'camera/image', 10) # latest frame

        # topic subscribers 
        self.create_subscription(Bool, 'camera/capture', self.capture_callback, 10) # trigger to capture latest frame

        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera.')
            return

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) # ensures that read() is always fresh

        self.br = CvBridge()
        self.latest_frame = None
        self.lock = threading.Lock()

        self.running = True
        self.capture_thread = threading.Thread(target=self._drain_camera, daemon=True)
        self.capture_thread.start()

        self.get_logger().info('Camera ready. Waiting for trigger...')

    # continuously drain the camera buffer in the background
    def _drain_camera(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest_frame = frame
            else:
                self.get_logger().warning('Camera read failed, retrying...')

    # callbacks
    def capture_callback(self, msg):
        if not msg.data:
            return

        with self.lock:
            frame = self.latest_frame.copy() if self.latest_frame is not None else None

        if frame is not None:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame, encoding='bgr8'))
            self.get_logger().info('Published latest frame.')
        else:
            self.get_logger().warning('Trigger received but no frame available yet.')

    def destroy_node(self):
        self.running = False
        self.capture_thread.join(timeout=2.0)
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Webcam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()