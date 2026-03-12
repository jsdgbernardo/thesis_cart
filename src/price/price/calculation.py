#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory
import json
import time
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge

from price.weight.weight import WeightDetector


class CartNode(Node):

    def __init__(self):
        super().__init__('cart_node')

        # load item database
        price_path = get_package_share_directory('price')
        json_path = os.path.join(price_path, 'items', 'grocery_items.json')

        try:
            with open(json_path, 'r') as f:
                self.items_data = json.load(f)
            self.get_logger().info('Database loaded successfully.')
        except FileNotFoundError:
            self.get_logger().error(f'File not found at {json_path}')
            self.items_data = {}
        except json.JSONDecodeError:
            self.get_logger().error('Error decoding JSON file.')
            self.items_data = {}

        self.weight_detector = WeightDetector()
        self.br = CvBridge()

        # state
        self.latest_detections = None       # most recent YOLO detections
        self.pending_weight_result = None   # weight event waiting to be resolved
        self.pending_trigger_id = 0         # increments each time a trigger is sent
        self.received_trigger_id = 0        # id of the trigger this annotated frame belongs to

        # ros2 subscriptions
        self.create_subscription(Image, 'camera/detected', self.detected_image_callback, 10)
        self.create_subscription(String, 'camera/detections', self.yolo_callback, 10)

        # ros2 publishers
        self.capture_publisher = self.create_publisher(Bool, 'camera/capture', 10)

        self.create_timer(0.1, self.loop)  # 10 Hz

        self.get_logger().info('Ready. Place items on scale.')

    # callbacks
    def detected_image_callback(self, msg):
        if self.pending_weight_result is None:
            return

        if self.received_trigger_id != self.pending_trigger_id:
            self.get_logger().debug(
                f'Ignoring stale annotated frame '
                f'(trigger {self.received_trigger_id} != {self.pending_trigger_id})'
            )
            return

        annotated = self.br.imgmsg_to_cv2(msg, 'bgr8')
        self._save_frame(annotated, self.pending_weight_result['action'])

        weight_item = self.detect_item_by_weight(self.pending_weight_result['weight'])
        combined = self.detect_item_combined(weight_item)

        if combined:
            name = combined.get('item_name', 'Unknown')
            confidence = combined.get('confidence', 'N/A')
            source = combined.get('source', 'N/A')
            self.get_logger().info(
                f"Item detected: {name} | confidence: {confidence} | source: {source}"
            )
        else:
            self.get_logger().info('No item matched.')

        
        self.pending_weight_result = None

    def yolo_callback(self, msg):
        try:
            self.latest_detections = json.loads(msg.data)
            self.received_trigger_id = self.pending_trigger_id
        except json.JSONDecodeError:
            self.get_logger().warning('Failed to parse YOLO detections.')
            self.latest_detections = None

    # ── Main loop ──────────────────────────────────────────────────────────────

    def loop(self):
        if self.pending_weight_result is not None:
            return

        weight_result = self.weight_detector.read_weight()
        if weight_result is None:
            return

        action = weight_result['action']
        delta = weight_result['weight']
        self.get_logger().info(
            f"Weight {'added' if action == 'added' else 'removed'}: {delta:.2f} g"
        )

        self.pending_trigger_id += 1
        self.pending_weight_result = weight_result
        self.latest_detections = None

        trigger = Bool()
        trigger.data = True
        self.capture_publisher.publish(trigger)
        self.get_logger().info(
            f'Capture requested.'
        )

    def detect_item_by_weight(self, delta_weight):
        abs_weight = abs(delta_weight)
        best_match = None
        smallest_diff = float('inf')

        for value in self.items_data.values():
            candidates = value if isinstance(value, list) else [value]

            for item in candidates:
                expected = item['expected_weight']
                tolerance = item['weight_tolerance']

                if expected == 0.0:
                    continue

                if (expected - tolerance) <= abs_weight <= (expected + tolerance):
                    diff = abs(abs_weight - expected)
                    if diff < smallest_diff:
                        smallest_diff = diff
                        best_match = item

        return best_match

    def detect_item_combined(self, weight_item):
        """
          1. weight + YOLO agree   → weight+yolo confidence
          2. weight alone          → weight-based confidence
          3. YOLO alone (>0.7)     → yolo confidence
          4. nothing               → None
        """
        detections = self.latest_detections
        yolo_available = (
            detections is not None
            and detections.get('classes')
            and detections.get('confidences')
        )

        if weight_item:
            if yolo_available:
                best_idx = detections['confidences'].index(max(detections['confidences']))
                best_conf = detections['confidences'][best_idx]
                yolo_class_names = detections.get('class_names', [])

                yolo_agrees = False
                if yolo_class_names:
                    yolo_name = yolo_class_names[best_idx]
                    weight_class = weight_item.get('yolo_class_id', '')
                    yolo_slug = yolo_name.strip().lower().replace(' ', '-')
                    weight_slug = weight_class.strip().lower()
                    yolo_agrees = (yolo_slug == weight_slug)

                if best_conf > 0.7 and yolo_agrees:
                    return {
                        **weight_item,
                        'confidence': round(best_conf, 2),
                        'source': 'weight+yolo',
                    }
                elif best_conf > 0.7:
                    yolo_name = yolo_class_names[best_idx] if yolo_class_names else '?'
                    self.get_logger().warning(
                        f"YOLO disagrees: weight says "
                        f"'{weight_item.get('yolo_class_id')}', "
                        f"YOLO says '{yolo_name}' ({best_conf:.2f})"
                    )
                    return {**weight_item, 'confidence': 'weight-based', 'source': 'weight'}
                else:
                    return {**weight_item, 'confidence': 'weight-based', 'source': 'weight'}
            else:
                return {**weight_item, 'confidence': 'weight-based', 'source': 'weight'}

        elif yolo_available:
            best_conf = max(detections['confidences'])
            if best_conf > 0.7:
                best_idx = detections['confidences'].index(best_conf)
                yolo_class_names = detections.get('class_names', [])
                detected_name = yolo_class_names[best_idx] if yolo_class_names else 'Unknown'
                return {
                    'item_name': detected_name,
                    'confidence': round(best_conf, 2),
                    'source': 'yolo',
                }

        return None

    # helpers
    def _save_frame(self, frame, action: str):
        ts = int(time.time())
        filename = f'/tmp/annotated_{action}_{ts}.jpg'
        try:
            cv2.imwrite(filename, frame)
            self.get_logger().info(f'Frame saved: {filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to save frame: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CartNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()