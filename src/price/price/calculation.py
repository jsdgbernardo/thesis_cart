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
# 1) detect_item_by weight -> match weight in database
# 2) identify_and_update -> weight + yolo -> continue
#                        -> weight only -> continue  
#                        -> yolo only -> continue
#                        -> confict -> trust weight
#                        -> last resort -> synch to current yolo view

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

class CartNode(Node):

    def __init__(self):
        super().__init__('cart_node')

        # acces item database
        price_path = get_package_share_directory('price')
        json_path = os.path.join(price_path, 'items', 'grocery_items.json')
        cart_path = os.path.join(price_path, 'items', 'cart.json')

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

        self.weight_detector = WeightSensor()
        self.br = CvBridge()

        # list of items in the cart
        self.items_in_cart = {}

        # for weight detection
        self.pending_weight_result = None
        self.pending_trigger_time = None
        self.pending_trigger_id = 0

        # for image detection
        self.current_annotated_frame = None
        self.current_detections = None
        self.current_image_trigger_id = -1
        self.current_detections_trigger_id = -1

        # topic subscribers
        self.create_subscription(Image, 'camera/detected', self.detected_image_callback, 10)
        self.create_subscription(String, 'camera/detections', self.detections_callback, 10)

        # topic publishers
        self.capture_publisher = self.create_publisher(Bool, 'camera/capture', 10)
        self.receipt_publisher = self.create_publisher(String, 'app/receipt', 10)

        self._write_cart_file()

        self.create_timer(0.1, self.loop)
        self.get_logger().info('Ready. Place items on scale.')

    # callbacks - store latest data
    def detected_image_callback(self, msg):
        self.current_annotated_frame = self.br.imgmsg_to_cv2(msg, 'bgr8')
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

    # resolution logic: weight -> image -> reset
    def _try_resolve(self):
        if self.pending_weight_result is None:
            return
        tid = self.pending_trigger_id
        if self.current_image_trigger_id == tid and self.current_detections_trigger_id == tid:
            self._resolve()

    def _resolve(self):
        weight_result = self.pending_weight_result
        action = weight_result['action']
        delta = weight_result['weight']

        if self.current_annotated_frame is not None:
            self._save_frame(self.current_annotated_frame, action)

        yolo_state = self._build_yolo_state(self.current_detections)
        result = self._identify_and_update(action, delta, yolo_state)

        if result:
            self.get_logger().info(
                f"Item {action}: {result.get('item_name', 'Unknown')} "
                f"| confidence: {result.get('confidence', 'N/A')} "
                f"| source: {result.get('source', 'N/A')} "
            )
            self._write_cart_file(delta)
        else:
            self.get_logger().info(
                f'No item matched.'
            )

        self.pending_weight_result = None
        self.pending_trigger_time = None

    def _build_yolo_state(self, detections) -> dict:
        state = {}
        if not detections:
            return state
        for name in detections.get('class_names', []):
            state[name] = state.get(name, 0) + 1
        return state

    def _identify_and_update(self, action: str, delta: float, yolo_state: dict):
        weight_item = self.detect_item_by_weight(delta)

        if action == 'added':
            return self._handle_added(weight_item, yolo_state)
        else:
            return self._handle_removed(weight_item, yolo_state)

    def _cart_add(self, item: dict, count: int = 1):
        key = item.get('yolo_class_id', 'Unknown')
        if key in self.items_in_cart:
            self.items_in_cart[key]['count'] += count
        else:
            self.items_in_cart[key] = {
                'item_name': item.get('item_name', 'Unknown'),
                'price': item.get('price', 0.0),
                'count': count,
            }

    def _cart_remove(self, key: str, count: int = 1):
        if key not in self.items_in_cart:
            return
        self.items_in_cart[key]['count'] -= count
        if self.items_in_cart[key]['count'] <= 0:
            self.items_in_cart.pop(key)

    # add / remove handlers
    def _handle_added(self, weight_item, yolo_state: dict):
        cart_name_counts = {
            v['item_name']: v['count'] for v in self.items_in_cart.values()
        }

        yolo_added = {}
        for name, count in yolo_state.items():
            old = cart_name_counts.get(name, 0)
            if count > old:
                yolo_added[name] = count - old

        if weight_item and yolo_added:
            weight_class = weight_item.get('yolo_class_id', '').strip().lower()
            for name, count in yolo_added.items():
                yolo_slug = name.strip().lower().replace(' ', '-')
                if yolo_slug == weight_class:
                    self._cart_add(weight_item, count)
                    return {**weight_item, 'source': 'both'}

            yolo_name = next(iter(yolo_added))
            self.get_logger().warning(
                f"Detection Conflict: Weight says '{weight_item.get('yolo_class_id')}', "
                f"YOLO says '{yolo_name}'."
                f"Trusting weight..."
            )
            self._sync_cart_to_yolo(yolo_state)
            return {**weight_item, 'source': 'weight'}

        elif weight_item:
            self._cart_add(weight_item)
            return {**weight_item, 'source': 'weight'}

        elif yolo_added:
            yolo_name = next(iter(yolo_added))
            self._cart_add(
                {'yolo_class_id': yolo_name, 'item_name': yolo_name, 'price': 0.0},
                yolo_added[yolo_name]
            )
            return {'item_name': yolo_name, 'source': 'yolo'}

        return None

    def _handle_removed(self, weight_item, yolo_state: dict):
        cart_name_counts = {
            v['item_name']: v['count'] for v in self.items_in_cart.values()
        }

        yolo_removed = {}
        for name, old_count in cart_name_counts.items():
            new_count = yolo_state.get(name, 0)
            if new_count < old_count:
                yolo_removed[name] = old_count - new_count

        if weight_item and yolo_removed:
            weight_class = weight_item.get('yolo_class_id', '').strip().lower()
            for name, count in yolo_removed.items():
                yolo_slug = name.strip().lower().replace(' ', '-')
                if yolo_slug == weight_class:
                    self._cart_remove(weight_item.get('yolo_class_id'), count)
                    return {**weight_item, 'source': 'both'}

            yolo_name = next(iter(yolo_removed))
            self.get_logger().warning(
                f"Remove conflict: weight says '{weight_item.get('yolo_class_id')}', "
                f"YOLO says '{yolo_name}' was removed. Trusting weight."
            )
            self._force_remove_by_weight(weight_item)
            return {**weight_item, 'source': 'weight'}

        elif weight_item:
            self.get_logger().info(
                f"Detection Conflict: YOLO still sees {yolo_name} after removal. "
                f"Trusting weight..."
            )
            self._force_remove_by_weight(weight_item)
            return {**weight_item, 'source': 'weight'}

        elif yolo_removed:
            yolo_name = next(iter(yolo_removed))
            self._cart_remove(yolo_name, yolo_removed[yolo_name])
            return {'item_name': yolo_name, 'source': 'yolo'}

        if weight_item:
            self.get_logger().warning('Could not identify removed item via YOLO. Trusting weight.')
            self._force_remove_by_weight(weight_item)
            return {**weight_item, 'source': 'weight'}

        self.get_logger().warning('Could not identify removed item. Syncing cart to YOLO detections.')
        self._sync_cart_to_yolo(yolo_state)
        return None

    def _force_remove_by_weight(self, weight_item):
        self._cart_remove(weight_item.get('yolo_class_id', ''))

    # last resort: rebuild cart.json from YOLO's current view
    def _sync_cart_to_yolo(self, yolo_state: dict):
        self.items_in_cart = {
            name: {'item_name': name, 'price': 0.0, 'count': count}
            for name, count in yolo_state.items()
        }

    # cart.json
    def _write_cart_file(self, delta_weight: float = 0.0):
        items = []
        for v in self.items_in_cart.values():
            if v.get('type') == 'produce':
                weight_kg = delta_weight / 1000.0
                subtotal = round(v['price'] * weight_kg, 2)
            else:
                subtotal = round(v['price'] * v['count'], 2)

            items.append({
                'item_name': v['item_name'],
                'price': v['price'],
                'count': v['count'],
                'subtotal': subtotal,
            })

        total = round(sum(i['subtotal'] for i in items), 2)
        payload = {
            'items': items,
            'total': total,
            'last_updated': time.strftime('%Y-%m-%dT%H:%M:%S'),
        }

        try:
            with open(cart_path, 'w') as f:
                json.dump(payload, f, indent=2)
            self.get_logger().info('cart.json updated')
        except Exception as e:
            self.get_logger().error(f'Failed to write cart file: {e}')

        receipt_msg = String()
        receipt_msg.data = json.dumps(payload)
        self.receipt_publisher.publish(receipt_msg)

    # detections
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
        delta = weight_result['weight']
        self.get_logger().info(
            f"Weight {'added' if action == 'added' else 'removed'}: {delta:.2f} g"
        )

        self.pending_trigger_id += 1
        self.current_annotated_frame = None
        self.current_detections = None
        self.current_image_trigger_id = -1
        self.current_detections_trigger_id = -1

        self.pending_weight_result = weight_result
        self.pending_trigger_time = time.time()

        trigger = Bool()
        trigger.data = True
        self.capture_publisher.publish(trigger)
        self.get_logger().info('Capture requested.')

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