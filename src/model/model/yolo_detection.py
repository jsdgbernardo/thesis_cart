#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO

import threading
import json
import cv2
import os

from ament_index_python.packages import get_package_share_directory


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            1
        )

        self.image_pub = self.create_publisher(Image, 'camera/detected', 10)
        self.det_pub = self.create_publisher(String, 'camera/detections', 10)

        model_dir = get_package_share_directory('model')
        model_path = os.path.join(model_dir, 'my_model_ncnn_model')

        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)

        self.latest_frame = None
        self.lock = threading.Lock()

        self.worker = threading.Thread(target=self.inference_loop)
        self.worker.daemon = True
        self.worker.start()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        with self.lock:
            self.latest_frame = frame

    def inference_loop(self):

        while rclpy.ok():

            if self.latest_frame is None:
                continue

            with self.lock:
                frame = self.latest_frame.copy()

            results = self.model(frame)

            annotated = results[0].plot()

            ros_image = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            self.image_pub.publish(ros_image)

            detections_data = {
                "classes": [int(c) for c in results[0].boxes.cls.tolist()],
                "confidences": [float(conf) for conf in results[0].boxes.conf.tolist()],
                "boxes": results[0].boxes.xyxy.tolist()
            }

            self.det_pub.publish(String(data=json.dumps(detections_data)))


def main():
    rclpy.init()
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
