#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory
import RPi.GPIO as GPIO
import json

from price.weight.weight import WeightDetector
# from price.camera.camera import CameraDetector

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
        # self.camera_detector = CameraDetector()

        self.timer = self.create_timer(0.25, self.loop)

        self.get_logger().info("Place items on scale")

    def loop(self):
        result = self.weight_detector.read_weight()

        if result:

            if result["action"] == "added":

                weight_item = self.detect_item_by_weight(result["weight"])
                delta = result["weight"]

                self.get_logger().info(f"Added: {delta:.2f} g")

                # weight-based detection
                if weight_item:
                    name = weight_item.get("item_name", "Unknown")

                    self.get_logger().info(
                        f"Weight Detected: {name}"
                    )
                else:
                    self.get_logger().info("No item detected by weight.")

                # # Camera detection
                # camera_name, confidence = self.camera_detector.detect_item()

                # if camera_name:
                #     self.get_logger().info(
                #         f"Camera Detected: {camera_name} | Conf: {confidence:.2f}"
                #     )
                # else:
                #     self.get_logger().info("No item detected by camera.")

                # comparison logic

            else:
                self.get_logger().info(
                    f"Removed: {result['weight']:.2f} g"
                )

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

def main(args=None):
    rclpy.init(args=args)
    node = CartNode()
    rclpy.spin(node)
    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()
