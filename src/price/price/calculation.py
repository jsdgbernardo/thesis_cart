#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory
import RPi.GPIO as GPIO
import json
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

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

        # subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.listener_callback,
            10
        )
        self.subscription

        self.br = CvBridge()
        # store latest frame for capture when weight changes
        self.last_frame = None

        self.get_logger().info("Place items on scale")
    
    def listener_callback(self, msg):
        self.get_logger().info("Receiving image")
        current_frame = self.br.imgmsg_to_cv2(msg)
        # keep a copy of the most recent frame
        self.last_frame = current_frame

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

            # capture an image at the moment of change
            self.capture_frame(weight_result["action"])

            weight_item = self.detect_item_by_weight(weight_result["weight"])
            if weight_item:
                name = weight_item.get("item_name", "Unknown")
                self.get_logger().info(f"Item: {name}")
            else:
                self.get_logger().info("No item detected by weight.")
        else:
            # nothing detected, still polling
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

    def capture_frame(self, action: str):
        if self.last_frame is None:
            self.get_logger().warning("No camera frame available to capture.")
            return

        # create a timestamped filename
        import time
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