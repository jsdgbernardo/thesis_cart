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
# 3) compare_detections     -> agree / partial / conflict / single-source
# 4) identify_and_update    -> weight + yolo agree   -> high confidence, finalize
#                           -> weight only            -> medium confidence, finalize
#                           -> yolo only              -> low confidence, finalize
#                           -> conflict               -> trust weight, warn
#                           -> neither                -> last resort, sync to yolo view

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

CONFIDENCE_HIGH     = 'high'      # weight and YOLO agree
CONFIDENCE_MEDIUM   = 'medium'    # weight match only
CONFIDENCE_LOW      = 'low'       # YOLO match only
CONFIDENCE_CONFLICT = 'conflict'  # both matched but disagree

class CartNode(Node):

    def __init__(self):
        super().__init__('cart_node')

        # access item database
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

        self._yolo_index = {}
        for value in self.items_data.values():
            candidates = value if isinstance(value, list) else [value]
            for item in candidates:
                cid = item.get('yolo_class_id', '').strip().lower()
                if cid:
                    self._yolo_index[cid] = item

        # items currently in the cart
        self.items_in_cart = {}

        self.weight_detector = WeightSensor(self)
        self.br = CvBridge()
        self.app = Receipt(self, self.cart_path, self.items_in_cart)

        # for weight detection
        self.pending_weight_result = None
        self.pending_trigger_time  = None
        self.pending_trigger_id    = 0

        # for image detection
        self.current_annotated_frame       = None
        self.current_detections            = None
        self.current_image_trigger_id      = -1
        self.current_detections_trigger_id = -1

        # topic subscribers
        self.create_subscription(Image,  'camera/detected',   self.detected_image_callback, 10)
        self.create_subscription(String, 'camera/detections', self.detections_callback,     10)

        # topic publishers
        self.capture_publisher = self.create_publisher(Bool,   'camera/capture', 10)
        self.receipt_publisher = self.create_publisher(String, 'app/receipt',    10)
        self.shopping_list_publisher = self.create_publisher(String, 'remove_items',    10)

        self.app.write_cart_file()

        self.create_timer(0.1, self.loop)
        self.get_logger().info('Ready. Place item on scale.')

    # callbacks 
    def detected_image_callback(self, msg):
        self.current_annotated_frame  = self.br.imgmsg_to_cv2(msg, 'bgr8')
        self.current_image_trigger_id = self.pending_trigger_id
        self._try_resolve()

    def detections_callback(self, msg):
        try:
            self.current_detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('Failed to parse detections.')
            self.current_detections = None
        self.current_detections_trigger_id = self.pending_trigger_id
        self._try_resolve()

    # resolution
    def _try_resolve(self):
        if self.pending_weight_result is None:
            return
        tid = self.pending_trigger_id
        if self.current_image_trigger_id == tid and self.current_detections_trigger_id == tid:
            self._resolve()

    def _resolve(self):
        weight_result = self.pending_weight_result
        action = weight_result['action']
        delta  = weight_result['weight']

        if self.current_annotated_frame is not None:
            self._save_frame(self.current_annotated_frame, action)

        yolo_state = self._build_yolo_state(self.current_detections)
        result     = self._identify_and_update(action, delta, yolo_state)

        if result:
            self.get_logger().info(
                f"Item {action}: {result.get('item_name', 'Unknown')} "
                f"| confidence: {result.get('confidence', 'N/A')} "
                f"| source: {result.get('source', 'N/A')} "
            )
        else:
            self.get_logger().info('No item matched.')

        self.app.write_cart_file(abs(delta))

        self.pending_weight_result = None
        self.pending_trigger_time  = None

    def _build_yolo_state(self, detections) -> dict:
        """Return {class_name: count} from raw detections."""
        state = {}
        if not detections:
            return state
        for name in detections.get('class_names', []):
            state[name] = state.get(name, 0) + 1
        return state

    # dual detection + comparison
    def _identify_and_update(self, action: str, delta: float, yolo_state: dict):
        weight_item = self.detect_item_by_weight(delta)
        yolo_item   = self.detect_item_by_yolo(yolo_state, action)

        comparison = self._compare_detections(weight_item, yolo_item)
        self.get_logger().info(
            f"Detection — "
            f"weight: '{weight_item.get('yolo_class_id') if weight_item else None}' | "
            f"yolo: '{yolo_item.get('yolo_class_id') if yolo_item else None}' | "
            f"result: {comparison['confidence']}"
        )

        if action == 'added':
            return self._handle_added(comparison, yolo_state)
        else:
            return self._handle_removed(comparison, yolo_state)

    def detect_item_by_weight(self, delta_weight):
        """Return the best-matching database item for the given weight delta."""
        abs_weight  = abs(delta_weight)
        best_match  = None
        smallest_diff = float('inf')

        for value in self.items_data.values():
            candidates = value if isinstance(value, list) else [value]
            for item in candidates:
                expected  = item['expected_weight']
                tolerance = item['weight_tolerance']
                if expected == 0.0:
                    continue
                if (expected - tolerance) <= abs_weight <= (expected + tolerance):
                    diff = abs(abs_weight - expected)
                    if diff < smallest_diff:
                        smallest_diff = diff
                        best_match    = item

        return best_match

    def detect_item_by_yolo(self, yolo_state: dict, action: str = 'added'):
        """Return the best-candidate database item from the current YOLO frame.

        For 'added':   prefer a class whose count exceeds what is in the cart.
        For 'removed': prefer a class whose count has dropped below what is in the cart.
        Falls back to the most frequently seen database class if no delta is found.
        """
        if not yolo_state:
            return None

        cart_name_counts = {
            v['item_name']: v['count'] for v in self.items_in_cart.values()
        }

        delta_candidates = {}

        if action == 'added':
            for name, count in yolo_state.items():
                slug = name.strip().lower().replace(' ', '-')
                if slug in self._yolo_index:
                    delta = count - cart_name_counts.get(name, 0)
                    if delta > 0:
                        delta_candidates[slug] = (delta, self._yolo_index[slug])
        else:  # removed
            for name, old_count in cart_name_counts.items():
                slug = name.strip().lower().replace(' ', '-')
                if slug in self._yolo_index:
                    new_count = yolo_state.get(name, 0)
                    delta = old_count - new_count
                    if delta > 0:
                        delta_candidates[slug] = (delta, self._yolo_index[slug])

        if delta_candidates:
            best = max(delta_candidates, key=lambda k: delta_candidates[k][0])
            return delta_candidates[best][1]

        # Fallback: most-seen class that exists in the database
        for name, _ in sorted(yolo_state.items(), key=lambda x: -x[1]):
            slug = name.strip().lower().replace(' ', '-')
            if slug in self._yolo_index:
                return self._yolo_index[slug]

        return None

    def _compare_detections(self, weight_item, yolo_item) -> dict:
        """Cross-check weight and YOLO detections.

        Returns:
            item        - the item to act on
            confidence  - CONFIDENCE_* constant
            source      - 'both' | 'weight' | 'yolo' | 'conflict'
            weight_item - raw weight result (may be None)
            yolo_item   - raw yolo result   (may be None)
        """
        base = {'weight_item': weight_item, 'yolo_item': yolo_item}

        if weight_item and yolo_item:
            w_class = weight_item.get('yolo_class_id', '').strip().lower()
            y_class = yolo_item.get('yolo_class_id',  '').strip().lower()

            if w_class == y_class:
                # Both sensors agree — highest confidence
                return {**base, 'item': weight_item,
                        'confidence': CONFIDENCE_HIGH, 'source': 'both'}
            else:
                # Disagreement — trust weight, surface the conflict
                self.get_logger().warning(
                    f"Conflict: weight='{w_class}' vs yolo='{y_class}'. "
                    f"Trusting weight."
                )
                return {**base, 'item': weight_item,
                        'confidence': CONFIDENCE_CONFLICT, 'source': 'conflict'}

        elif weight_item:
            return {**base, 'item': weight_item,
                    'confidence': CONFIDENCE_MEDIUM, 'source': 'weight'}

        elif yolo_item:
            return {**base, 'item': yolo_item,
                    'confidence': CONFIDENCE_LOW, 'source': 'yolo'}

        return {**base, 'item': None, 'confidence': None, 'source': None}

    # cart management
    def _cart_add(self, item: dict, count: int = 1):
        key = item.get('yolo_class_id', 'Unknown')
        if key in self.items_in_cart:
            self.items_in_cart[key]['count'] += count
        else:
            self.items_in_cart[key] = {
                'item_name': item.get('item_name', 'Unknown'),
                'price':     item.get('price', 0.0),
                'count':     count,
            }

    def _cart_remove(self, key: str, count: int = 1):
        if key not in self.items_in_cart:
            return
        self.items_in_cart[key]['count'] -= count
        if self.items_in_cart[key]['count'] <= 0:
            self.items_in_cart.pop(key)

    def _force_remove_by_weight(self, weight_item):
        self._cart_remove(weight_item.get('yolo_class_id', ''))

    def _sync_cart_to_yolo(self, yolo_state: dict):
        """Last resort: rebuild items_in_cart from YOLO's current view."""
        self.items_in_cart = {
            name: {'item_name': name, 'price': 0.0, 'count': count}
            for name, count in yolo_state.items()
        }

    # add / remove handlers
    def _handle_added(self, comparison: dict, yolo_state: dict):
        item       = comparison['item']
        confidence = comparison['confidence']
        source     = comparison['source']

        if item is None:
            self.get_logger().warning('Add: neither sensor identified an item.')
            return None

        if confidence == CONFIDENCE_CONFLICT:
            # Weight and YOLO disagree — trust weight, re-sync visual state
            self._cart_add(item)
            self._sync_cart_to_yolo(yolo_state)
            return {**item, 'confidence': confidence, 'source': 'weight'}

        # high / medium / low — finalize normally
        self._cart_add(item)
        return {**item, 'confidence': confidence, 'source': source}

    def _handle_removed(self, comparison: dict, yolo_state: dict):
        item       = comparison['item']
        confidence = comparison['confidence']
        source     = comparison['source']

        if item is None:
            self.get_logger().warning(
                'Remove: neither sensor identified an item. '
                'Syncing cart to YOLO view.'
            )
            self._sync_cart_to_yolo(yolo_state)
            return None

        if confidence == CONFIDENCE_CONFLICT:
            # Disagreement — trust weight
            self._force_remove_by_weight(comparison['weight_item'])
            return {**item, 'confidence': confidence, 'source': 'weight'}

        if confidence == CONFIDENCE_LOW:
            # YOLO only — remove by YOLO class key
            self._cart_remove(item.get('yolo_class_id', ''))
            return {**item, 'confidence': confidence, 'source': source}

        # high / medium — weight confirmed, remove cleanly
        self._force_remove_by_weight(item)
        return {**item, 'confidence': confidence, 'source': source}

    # main loop
    def loop(self):
        if self.pending_weight_result is not None:
            elapsed = time.time() - self.pending_trigger_time
            if elapsed > 5.0:
                self.get_logger().warning(
                    f'Timeout after {elapsed:.1f}s. Resolving with partial data. '
                    f'image_ready={self.current_image_trigger_id == self.pending_trigger_id}, '
                    f'detections_ready={self.current_detections_trigger_id == self.pending_trigger_id}'
                )
                self._resolve()
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

    # helpers
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