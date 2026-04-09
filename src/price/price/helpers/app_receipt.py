import time
import json
from std_msgs.msg import String

class Receipt:
    
    def __init__(self, node, cart_path: str, items_in_cart: dict):
        self.get_logger = node.get_logger

        # access cart.json
        self.cart_path = cart_path

        # items currently in the cart
        self.items_in_cart = items_in_cart

        # topic subscribers
        self.receipt_publisher = node.create_publisher(String, 'app/receipt', 10)

    def write_cart_file(self, delta_weight: float = 0.0):
        items = []        

        # price calculation
        for v in self.items_in_cart.values():
            if v.get('item_type') == 'produce':
                weight_kg = v['weight_g'] / 1000.0
                v['subtotal'] = round(v['price'] * weight_kg, 2)
            else:
                v['subtotal'] = round(v['price'] * v['count'], 2)

            try: 
                items.append({
                    'item_name': v['item_name'],
                    'price':     v['price'],
                    'count':     v['count'],
                    'subtotal':  v['subtotal'],
                })
                self.get_logger().info(f'Price calculated for {v["item_name"]}: P{v["subtotal"]}')
            except KeyError as e:
                self.get_logger().error(f'Missing key in item data: {e} - {v}')

        total   = round(sum(i['subtotal'] for i in items), 2)
        payload = {
            'items':        items,
            'total':        total,
            'last_updated': time.strftime('%Y-%m-%dT%H:%M:%S'),
        }
        
        try:
            with open(self.cart_path, 'w') as f:
                json.dump(payload, f, indent=2)
            self.get_logger().info(f'Receipt updated. {self.cart_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to write cart file: {e}')

        receipt_msg      = String()
        receipt_msg.data = json.dumps(payload)
        self.receipt_publisher.publish(receipt_msg)