#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import RPi.GPIO as GPIO
from hx711 import HX711
import json 
import os
from ament_index_python.packages import get_package_share_directory
# from price.camera_detect import CameraDetector

class WeightScaleNode(Node):
    
    def __init__(self):
        super().__init__('weight_scale_node')

        # HX711 setup
        self.hx = HX711(5, 6)  # DOUT pin 5, SCK pin 6
        self.hx.reset()
        
        # calibration
        self.reference_unit = -115.50 
        self.tare_value = None
        
        time.sleep(0.5)  # wait for HX711 to stabilize
        
        # Read initial values for taring
        raw_data = self.hx.get_raw_data(times=5)
        if raw_data:
            self.tare_value = sum(raw_data) / len(raw_data)
        else:
            self.tare_value = 0

        self.get_logger().info("Place items on scale")
        self.get_logger().info("CTRL+C to exit")

        self.threshold = 5 
        self.prev_weight = self._get_weight()

        # run every 0.5 seconds
        self.timer = self.create_timer(0.25, self.read_weight)

        self.stability_samples = 5          # number of readings to check
        self.stability_tolerance = 2.0      # grams variation allowed
        self.readings_buffer = []
        self.change_detected = False
        self.start_weight = None

        # access JSON file
        price_path = get_package_share_directory('price')
        json_path = os.path.join(price_path,'items','grocery_items.json')
        
        try:
            with open(json_path, "r") as f:
                self.items_data = json.load(f)
                self.get_logger().info("Grocery items loaded successfully.")
        except FileNotFoundError:
            self.get_logger().error(f"File not found at {json_path}")
            self.items_data = {"items": []}
        except json.JSONDecodeError:
            self.get_logger().error("Error decoding JSON file.")
            self.items_data = {"items": []}

        # self.camera_detector = CameraDetector()

    def _get_weight(self):
        raw_data = self.hx.get_raw_data(times=5)
        if raw_data:
            average = sum(raw_data) / len(raw_data)
            weight_grams = (average - self.tare_value) / self.reference_unit
            return weight_grams
        return 0

    def read_weight(self):
        current_weight = self._get_weight()
        self.readings_buffer.append(current_weight)

        # Keep only last N samples
        if len(self.readings_buffer) > self.stability_samples:
            self.readings_buffer.pop(0)

        # Detect initial big change
        delta_weight = current_weight - self.prev_weight

        if not self.change_detected and abs(delta_weight) > self.threshold:
            self.change_detected = True
            self.start_weight = self.prev_weight
            self.get_logger().info("Weight change detected, waiting for stabilization...")

        # If change detected, wait until stable
        if self.change_detected and len(self.readings_buffer) == self.stability_samples:
            max_w = max(self.readings_buffer)
            min_w = min(self.readings_buffer)

            if abs(max_w - min_w) < self.stability_tolerance:
                final_weight = sum(self.readings_buffer) / len(self.readings_buffer)
                total_delta = final_weight - self.start_weight

                if total_delta > 0:
                    self.get_logger().info(f"Added: {total_delta:.2f} g")

                    # detect item by weight
                    detected_item = self.detect_item_by_weight(total_delta)

                    if detected_item:
                        name = detected_item.get("item_name", "Unknown")
                        price = detected_item.get("price", 0.0)

                        self.get_logger().info(
                            f"Weight Detected Item: {name} | Price: ₱{price}"
                        )
                    else:
                        self.get_logger().info("No matching item found.")

                    # detect item via camera
                    # camera_name, confidence = self.camera_detector.detect_item()
                    # if camera_name:
                    #     self.get_logger().info(
                    #         f"Camera Detected Item: {camera_name} | Confidence: {confidence:.2f}"
                    #     )
                    # else:
                    #     self.get_logger().info("No item detected by camera.")

                    # check if camera and weight detections are the same, if not, retry detection

                    # place all items in array

                else:
                    self.get_logger().info(f"Removed: {abs(total_delta):.2f} g")

                # Reset detection
                self.change_detected = False
                self.readings_buffer.clear()

        self.prev_weight = current_weight

        self.hx.power_down()
        self.hx.power_up()

    def detect_item_by_weight(self, delta_weight):
        abs_weight = abs(delta_weight)

        best_match = None
        smallest_diff = float("inf")

        for category, value in self.items_data.items():

            # Case 1: Category contains a list of items
            if isinstance(value, list):
                items_list = value
            else:
                # Case 2: Category contains a single item (dict)
                items_list = [value]

            for item in items_list:
                expected = item["expected_weight"]
                tolerance = item["weight_tolerance"]

                lower_bound = expected - tolerance
                upper_bound = expected + tolerance

                if lower_bound <= abs_weight <= upper_bound:
                    diff = abs(abs_weight - expected)

                    if diff < smallest_diff:
                        smallest_diff = diff
                        best_match = item

        return best_match

def main(args=None):
    rclpy.init(args=args)
    node = WeightScaleNode()
    rclpy.spin(node)
    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()
