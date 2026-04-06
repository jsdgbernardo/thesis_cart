#!/usr/bin/env python3

# Overall Flow:
# weight change -> read weight -> no change -> idle
#                              -> yes change -> store weight result
#                                            -> camera/capture = True
#
# camera/detected   -> image_callback      -> store annotated frame -> try_resolve
# camera/detections -> detections_callback -> store yolo detections -> try_resolve
#
# try_resolve -> both callbacks received -> resolve
#             -> timeout
#
# resolve:
# 1) detect_item_by_weight  -> match weight in database
# 2) detect_item_by_yolo    -> match top YOLO detection in database
# 3) compare_detections     -> agree / weight-only / yolo-only / conflict
# 4) identify_and_update    -> weight + yolo agree  -> HIGH,   finalize
#                           -> weight only           -> MEDIUM, finalize
#                           -> yolo only             -> MEDIUM, finalize (peer sensor)
#                           -> conflict              -> pick more specific match, warn
#                           -> neither               -> last resort, sync to yolo view

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

from price.weight.weight import WeightSensor
from price.helpers.app_receipt import Receipt

# Weight and YOLO are peers — neither is primary.
# HIGH   = both agree (maximum trust)
# MEDIUM = exactly one sensor fired (either is sufficient alone)
# CONFLICT = both fired but disagree (needs tie-breaking logic)
CONFIDENCE_HIGH     = 'high'
CONFIDENCE_MEDIUM   = 'medium'
CONFIDENCE_CONFLICT = 'conflict'


class CartNode(Node):

    def __init__(self):
        super().__init__('cart_node')

        # Access item database
        price_path = get_package_share_directory('price')
        json_path  = os.path.join(price_path, 'items', 'grocery_items.json')
        self.cart_path = os.path.join(price_path, 'items', 'cart.json')

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

        # Build YOLO class-id index for fast lookup
        self.yolo_index = {}
        for value in self.items_data.values():
            candidates = value if isinstance(value, list) else [value]
            for item in candidates:
                cid = item.get('yolo_class_id', '').strip().lower()
                if cid:
                    self.yolo_index[cid] = item

        self.items_in_cart = {}

        self.weight_detector = WeightSensor(self)
        self.br = CvBridge()
        self.app = Receipt(self, self.cart_path, self.items_in_cart)

        # Pending detection state
        self.pending_weight_result = None
        self.pending_trigger_time  = None
        self.pending_trigger_id    = 0

        self.current_annotated_frame       = None
        self.current_detections            = None
        self.current_image_trigger_id      = -1
        self.current_detections_trigger_id = -1

        # Topic subscribers
        self.create_subscription(Image,  'camera/detected',   self.detected_image_callback, 10)
        self.create_subscription(String, 'camera/detections', self.detections_callback,     10)

        # Topic publishers
        self.capture_publisher       = self.create_publisher(Bool,   'camera/capture', 10)
        self.receipt_publisher       = self.create_publisher(String, 'app/receipt',    10)
        self.shopping_list_publisher = self.create_publisher(String, 'remove_items',   10)

        self.app.write_cart_file()

        self.create_timer(0.1, self.loop)
        self.get_logger().info('Ready. Place item on scale.')

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def detected_image_callback(self, msg):
        self.current_annotated_frame  = self.br.imgmsg_to_cv2(msg, 'bgr8')
        self.current_image_trigger_id = self.pending_trigger_id
        self.try_resolve()

    def detections_callback(self, msg):
        try:
            self.current_detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Failed to parse detections JSON.')
            self.current_detections = None
        self.current_detections_trigger_id = self.pending_trigger_id
        self.try_resolve()

    # ------------------------------------------------------------------ #
    # Resolution
    # ------------------------------------------------------------------ #

    def try_resolve(self):
        if self.pending_weight_result is None:
            return
        tid = self.pending_trigger_id
        if self.current_image_trigger_id == tid and self.current_detections_trigger_id == tid:
            self.resolve()

    def resolve(self):
        weight_result = self.pending_weight_result
        action = weight_result['action']
        delta  = weight_result['weight']

        if self.current_annotated_frame is not None:
            self._save_frame(self.current_annotated_frame, action)

        yolo_state = self.build_yolo_state(self.current_detections)
        result     = self.identify_and_update(action, delta, yolo_state)

        if result:
            self.app.write_cart_file(abs(delta))
            self.get_logger().info(
                f"Item {action}: {result.get('item_name', 'Unknown')} "
                f"| confidence: {result.get('confidence', 'N/A')} "
                f"| source: {result.get('source', 'N/A')}"
            )
        else:
            self.get_logger().info('No item matched.')

        self.pending_weight_result = None
        self.pending_trigger_time  = None

    def build_yolo_state(self, detections) -> dict:
        state = {}
        if not detections:
            return state
        for name in detections.get('class_names', []):
            state[name] = state.get(name, 0) + 1
        return state

    # ------------------------------------------------------------------ #
    # Dual detection + comparison
    # ------------------------------------------------------------------ #

    def identify_and_update(self, action: str, delta: float, yolo_state: dict):
        weight_item = self.detect_item_by_weight(delta)
        yolo_item, yolo_delta = self.detect_item_by_yolo(yolo_state, action)

        comparison = self.compare_detections(weight_item, yolo_item)
        self.get_logger().info(
            f"Detection — "
            f"weight: '{weight_item.get('yolo_class_id') if weight_item else None}' | "
            f"yolo: '{yolo_item.get('yolo_class_id') if yolo_item else None}' | "
            f"result: {comparison['confidence']}"
        )

        if action == 'added':
            return self.handle_added(comparison, yolo_state, yolo_delta, weight_g=abs(delta))
        else:
            return self.handle_removed(comparison, yolo_state)

    def detect_item_by_weight(self, delta_weight):
        """Return best-matching item by weight, or None."""
        abs_weight    = abs(delta_weight)
        best_match    = None
        smallest_diff = float('inf')

        for value in self.items_data.values():
            candidates = value if isinstance(value, list) else [value]
            for item in candidates:
                expected  = item.get('expected_weight')
                tolerance = item.get('weight_tolerance', 0)
                if expected is None or expected == 0.0:
                    continue
                if (expected - tolerance) <= abs_weight <= (expected + tolerance):
                    diff = abs(abs_weight - expected)
                    if diff < smallest_diff:
                        smallest_diff = diff
                        best_match    = item

        return best_match

    def detect_item_by_yolo(self, yolo_state: dict, action: str = 'added'):
        """Return (best-matching item, delta count) from YOLO detections, or (None, 0)."""
        if not yolo_state:
            return None, 0

        cart_name_counts = {
            v['item_name']: v['count'] for v in self.items_in_cart.values()
        }

        delta_candidates = {}

        if action == 'added':
            for name, count in yolo_state.items():
                slug = name.strip().lower().replace(' ', '-')
                if slug in self.yolo_index:
                    delta = count - cart_name_counts.get(name, 0)
                    if delta > 0:
                        delta_candidates[slug] = (delta, self.yolo_index[slug])
        else:  # removed
            for name, old_count in cart_name_counts.items():
                slug = name.strip().lower().replace(' ', '-')
                if slug in self.yolo_index:
                    new_count = yolo_state.get(name, 0)
                    delta = old_count - new_count
                    if delta > 0:
                        delta_candidates[slug] = (delta, self.yolo_index[slug])

        if delta_candidates:
            best = max(delta_candidates, key=lambda k: delta_candidates[k][0])
            delta, item = delta_candidates[best]
            return item, delta

        # Fallback: top detection in the frame, assume delta of 1
        for name in sorted(yolo_state, key=lambda n: -yolo_state[n]):
            slug = name.strip().lower().replace(' ', '-')
            if slug in self.yolo_index:
                return self.yolo_index[slug], 1

        return None, 0

    def compare_detections(self, weight_item, yolo_item) -> dict:
        """
        Compare weight and YOLO as peer sensors.

        HIGH     — both agree on the same item.
        MEDIUM   — exactly one sensor returned a match (the other failed or timed out).
        CONFLICT — both matched, but to different items.
        """
        base = {'weight_item': weight_item, 'yolo_item': yolo_item}

        if weight_item and yolo_item:
            w_class = weight_item.get('yolo_class_id', '').strip().lower()
            y_class = yolo_item.get('yolo_class_id',  '').strip().lower()

            if w_class == y_class:
                # Perfect agreement — highest trust
                return {**base, 'item': yolo_item,
                        'confidence': CONFIDENCE_HIGH, 'source': 'both'}

            # Disagreement — both fired but identified different items.
            # Prefer the item with a tighter weight match when available;
            # fall back to weight (more deterministic than visual class).
            self.get_logger().warning(
                f"Sensor conflict: weight='{w_class}' vs yolo='{y_class}'. "
                f"Using weight as tiebreaker."
            )
            return {**base, 'item': weight_item,
                    'confidence': CONFIDENCE_CONFLICT, 'source': 'conflict-weight-wins'}

        elif weight_item:
            # Weight sensor matched; YOLO missed. Medium confidence — one peer caught it.
            return {**base, 'item': weight_item,
                    'confidence': CONFIDENCE_MEDIUM, 'source': 'weight'}

        elif yolo_item:
            # YOLO matched; weight missed. Medium confidence — one peer caught it.
            return {**base, 'item': yolo_item,
                    'confidence': CONFIDENCE_MEDIUM, 'source': 'yolo'}

        # Neither sensor matched
        return {**base, 'item': None, 'confidence': None, 'source': None}

    # ------------------------------------------------------------------ #
    # Cart management
    # ------------------------------------------------------------------ #

    def cart_add(self, item: dict, count: int = 1, weight_g: float = 0.0):
        key = item.get('yolo_class_id', 'Unknown')
        if key in self.items_in_cart:
            self.items_in_cart[key]['count']    += count
            self.items_in_cart[key]['weight_g'] += weight_g
        else:
            self.items_in_cart[key] = {
                'item_name': item.get('item_name', 'Unknown'),
                'item_type': item.get('item_type', 'normal'),
                'price':     item.get('price', 0.0),
                'count':     count,
                'weight_g':  weight_g,
            }

    def cart_remove(self, key: str, count: int = 1):
        if key not in self.items_in_cart:
            return
        self.items_in_cart[key]['count'] -= count
        if self.items_in_cart[key]['count'] <= 0:
            self.items_in_cart.pop(key)

    def sync_cart_to_yolo(self, yolo_state: dict):
        """Last resort: rebuild cart state entirely from YOLO's current view."""
        self.items_in_cart = {
            name: {'item_name': name, 'price': 0.0, 'count': count}
            for name, count in yolo_state.items()
        }

    # ------------------------------------------------------------------ #
    # Add / remove handlers
    # ------------------------------------------------------------------ #

    def handle_added(self, comparison: dict, yolo_state: dict,
                     yolo_delta: int = 1, weight_g: float = 0.0):
        item       = comparison['item']
        confidence = comparison['confidence']
        source     = comparison['source']

        if item is None:
            return None

        # For conflict, trust weight (already resolved in compare_detections).
        # For medium from YOLO, yolo_delta gives the best count estimate.
        # For medium from weight or high confidence, count = 1 (weight fired once).
        if source in ('yolo', 'conflict-weight-wins'):
            self.cart_add(item, count=yolo_delta, weight_g=weight_g)
        else:
            self.cart_add(item, count=1, weight_g=weight_g)

        return {**item, 'confidence': confidence, 'source': source}

    def handle_removed(self, comparison: dict, yolo_state: dict):
        item       = comparison['item']
        confidence = comparison['confidence']
        source     = comparison['source']

        if item is None:
            self.get_logger().warning(
                'Remove: neither sensor identified an item. '
                'Syncing cart to YOLO view as fallback.'
            )
            self.sync_cart_to_yolo(yolo_state)
            return None

        # In all non-None cases, we have a best-guess item to remove.
        # Conflict is already resolved to weight_item in compare_detections.
        self.cart_remove(item.get('yolo_class_id', ''))
        return {**item, 'confidence': confidence, 'source': source}

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #

    def loop(self):
        if self.pending_weight_result is not None:
            elapsed = time.time() - self.pending_trigger_time
            if elapsed > 5.0:
                self.get_logger().warning(
                    f'Timeout after {elapsed:.1f}s. Resolving with partial data. '
                    f'image_ready={self.current_image_trigger_id == self.pending_trigger_id}, '
                    f'detections_ready={self.current_detections_trigger_id == self.pending_trigger_id}'
                )
                self.resolve()
            return

        weight_result = self.weight_detector.read_weight()
        if weight_result is None:
            return

        action = weight_result['action']
        delta  = weight_result['weight']
        self.get_logger().info(
            f"Weight {'added' if action == 'added' else 'removed'}: {delta:.2f} g"
        )

        self.pending_trigger_id += 1
        self.current_annotated_frame       = None
        self.current_detections            = None
        self.current_image_trigger_id      = -1
        self.current_detections_trigger_id = -1

        self.pending_weight_result = weight_result
        self.pending_trigger_time  = time.time()

        trigger      = Bool()
        trigger.data = True
        self.capture_publisher.publish(trigger)
        self.get_logger().info('Capture requested.')

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _save_frame(self, frame, action: str):
        ts       = int(time.time())
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