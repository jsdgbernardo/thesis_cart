import rclpy
from rclpy.node import Node
import time
import json
import os
from hx711 import HX711

class WeightDetector:

    def __init__(self):
        # HX711 setup
        self.hx = HX711(5, 6)
        self.hx.reset()

        # calibration 
        self.reference_unit = -115.50
        self.threshold = 5
        self.stability_samples = 5
        self.stability_tolerance = 2.0
        json_path = None

        self.readings_buffer = []
        self.change_detected = False
        self.start_weight = None

        time.sleep(0.5)

        # for taring
        raw_data = self.hx.get_raw_data(times=5)
        if raw_data:
            self.tare_value = sum(raw_data) / len(raw_data)
        else:
            self.tare_value = 0

        self.prev_weight = self.get_weight()

        # load grocery items from JSON
        self.items_data = {"items": []}
        if json_path and os.path.exists(json_path):
            with open(json_path, "r") as f:
                self.items_data = json.load(f)

    def get_weight(self):
        raw_data = self.hx.get_raw_data(times=5)
        if raw_data:
            average = sum(raw_data) / len(raw_data)
            weight = (average - self.tare_value) / self.reference_unit
            return weight
        return 0

    def read_weight(self):
        current_weight = self.get_weight()
        self.readings_buffer.append(current_weight)

        if len(self.readings_buffer) > self.stability_samples:
            self.readings_buffer.pop(0)

        # initial big change
        delta_weight = current_weight - self.prev_weight

        result = None

        if not self.change_detected and abs(delta_weight) > self.threshold:
            self.change_detected = True
            self.start_weight = self.prev_weight
            self.get_logger().info("Weight change detected, waiting for stabilization...")

        if self.change_detected and len(self.readings_buffer) == self.stability_samples:
            max_w = max(self.readings_buffer)
            min_w = min(self.readings_buffer)

            if abs(max_w - min_w) < self.stability_tolerance:
                final_weight = sum(self.readings_buffer)/len(self.readings_buffer)
                total_delta = final_weight - self.start_weight

                if total_delta > 0:
                    item = self.detect_item_by_weight(total_delta)
                    result = {
                        "action": "added",
                        "weight": total_delta,
                        "item": item
                    }
                else:
                    result = {
                        "action": "removed",
                        "weight": abs(total_delta),
                        "item": item
                    }

                self.change_detected = False
                self.readings_buffer.clear()

        self.prev_weight = current_weight

        # reset detection
        self.hx.power_down()
        self.hx.power_up()

        return result

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
    node = WeightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()   