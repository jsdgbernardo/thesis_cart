#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory
import RPi.GPIO as GPIO
import json
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2
import time

from price.weight.weight import WeightDetector

class CartNode(Node):

    def __init__(self):
        super().__init__('cart_node')
        
        price_path = get_package_share_directory('price')
        json_path = os.path.join(price_path, 'items', 'grocery_items.json')

        try:
            with open(json_path, "r") as f:
                self.items_data = json.load(f)
                self.get_logger().info("Database loaded successfully.")
        except FileNotFoundError:
            self.get_logger().error(f"File not found at {json_path}")
            self.items_data = {"items": []}
        except json.JSONDecodeError:
            self.get_logger().error("Error decoding JSON file.")
            self.items_data = {"items": []}

        self.weight_detector = WeightDetector()

        self.timer = self.create_timer(0.1, self.loop)

        self.image_subscription = self.create_subscription(
            Image,
            'camera/image',
            self.listener_callback,
            10
        )
        self.image_subscription

        self.capture_publisher = self.create_publisher(
            Bool,
            'camera/capture',
            10
        )

        self.detection_subscription = self.create_subscription(
            Image,
            'camera/detected',
            self.detection_callback,
            10
        )
        self.detected_frame = None

        self.yolo_subscription = self.create_subscription(
            String,
            'camera/detections',
            self.yolo_callback,
            10
        )
        self.latest_detections = None

        self.br = CvBridge()
        self.last_frame = None

        self.get_logger().info("Place items on scale")
    
    def detection_callback(self, msg):
        self.detected_frame = self.br.imgmsg_to_cv2(msg)

    def yolo_callback(self, msg):
        try:
            self.latest_detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Failed to parse YOLO detections")
            self.latest_detections = None

    def listener_callback(self, msg):
        self.get_logger().info("Receiving captured frame")
        current_frame = self.br.imgmsg_to_cv2(msg)
        self.last_frame = current_frame
        self.capture_frame("Detected")
        self.last_frame = None

    def loop(self):
        weight_result = self.weight_detector.read_weight()

        if weight_result:
            # weight-based detection
            if weight_result["action"] == "added":
                delta = weight_result["weight"]
                self.get_logger().info(f"Added: {delta:.2f} g")
            else:
                delta = weight_result["weight"]
                self.get_logger().info(f"Removed: {delta:.2f} g")

            # trigger camera to send one frame
            trigger_msg = Bool()
            trigger_msg.data = True
            self.capture_publisher.publish(trigger_msg)

            self.get_logger().info("Capture requested")

            weight_item = self.detect_item_by_weight(weight_result["weight"])
            combined_item = self.detect_item_combined(weight_item)
            if combined_item:
                name = combined_item.get("item_name", "Unknown")
                confidence = combined_item.get("confidence", "Unknown")
                self.get_logger().info(f"Item: {name} (confidence: {confidence})")
            else:
                self.get_logger().info("No item detected.")
        else:
            self.get_logger().debug("No stable change detected yet.")

    def detect_item_by_weight(self, delta_weight):
        abs_weight = abs(delta_weight)
        best_match = None
        smallest_diff = float("inf")

        for category, value in self.items_data.items():

            if isinstance(value, list):
                items_list = value
            else:
                items_list = [value]

            for item in items_list:
                expected = item["expected_weight"]
                tolerance = item["weight_tolerance"]

                if (expected - tolerance) <= abs_weight <= (expected + tolerance):
                    diff = abs(abs_weight - expected)
                    if diff < smallest_diff:
                        smallest_diff = diff
                        best_match = item

        return best_match

    def detect_item_combined(self, weight_item):
        if self.latest_detections is None:
            return weight_item

        detections = self.latest_detections
        if not detections.get("classes") or not detections.get("confidences"):
            return weight_item

        # If weight-based detection found an item, verify with YOLO
        if weight_item:
            # Get the highest confidence YOLO detection
            if detections["confidences"]:
                max_confidence = max(detections["confidences"])
                if max_confidence > 0.7: 
                    # High confidence in YOLO, add it to the result
                    weight_item["confidence"] = round(max_confidence, 2)
                    weight_item["yolo_verified"] = True
                    self.get_logger().info(f"Item verified by YOLO with confidence {max_confidence:.2f}")
                    return weight_item
                else:
                    weight_item["confidence"] = "weight-based"
                    weight_item["yolo_verified"] = False
                    return weight_item
            else:
                weight_item["confidence"] = "weight-based"
                return weight_item
        else:
            # No weight-based match, try YOLO alone if confidence is high
            if detections["confidences"]:
                max_confidence = max(detections["confidences"])
                if max_confidence > 0.7:
                    self.get_logger().info(f"Item detected by YOLO with confidence {max_confidence:.2f}")
                    return {"item_name": "YOLO Detection", "confidence": round(max_confidence, 2)}
            return None

    def capture_frame(self, action: str):
        if self.last_frame is None:
            self.get_logger().warning("No camera frame available to capture.")
            return

        ts = int(time.time())
        filename = f"/tmp/weight_change_{action}_{ts}.jpg"
        try:
            cv2.imwrite(filename, self.last_frame)
            self.get_logger().info(f"Captured frame due to weight {action}: {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to write image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CartNode()
    rclpy.spin(node)
    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()