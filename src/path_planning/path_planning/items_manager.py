# Manager for selected items in the path planning nodes
# Updates selected items
# Allows for dynamic updates to the selected items without needing to directly modify the path planning logic

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

import json
import os

class ItemsManager(Node):
    def __init__(self):
        super().__init__('items_manager_node')

        self.selected_items = [] # Shopping list
        self.item_names = self.load_items()

        # Subscription to selected items topic (add item to the list)
        self.add_item = self.create_subscription(
            String,
            'add_items',
            self.add_items_callback,
            10
        )

        # Subscription to remove items topic (remove item from the list)
        self.remove_item = self.create_subscription(
            String,
            'shopping_cart',
            self.remove_item_callback,
            10
        )

        # Publisher for the current shopping list
        self.shopping_list = self.create_publisher(String, 'shopping_list', 10)

    # Load items from JSON file
    def load_items(self):
        price_path = get_package_share_directory('price')
        json_path = os.path.join(price_path,'items','grocery_items.json')
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)

            item_names = list(data.keys())
            return item_names
        
        except Exception as e:
            self.get_logger().error(f'Error loading items from JSON: {e}')
            return []

    # Callback for adding items to the shopping list
    def add_items_callback(self, msg):
        try:
            item = msg.data.strip()
            if not item:
                self.get_logger().warn('Empty item received')
                return
                
            if item in self.item_names and item not in self.selected_items:
                self.selected_items.append(item)
                
                # Publish the updated shopping list
                self.shopping_list.publish(
                    String(data=",".join(self.selected_items))
                )

                self.get_logger().info(f'Added item: {item}. Current shopping list: {self.selected_items}')

            elif item in self.selected_items:
                self.get_logger().info(f'Item already selected: {item}')
            else:
                self.get_logger().warn(f'Unknown item received: {item}. Available items: {self.item_names}')
        except Exception as e:
            self.get_logger().error(f'Error in add_items_callback: {e}', exc_info=True)

    # Callback for removing items from the shopping list
    def remove_item_callback(self, msg):
        item = msg.data
        if item in self.selected_items:
            self.selected_items.remove(item)

    # Publish the current shopping list to the topic
    # def publish_selected_items(self):
        # for item_name in self.selected_items:
        #     if item_name in self.items:
        #         item_info = self.items[item_name]
        #         x = item_info.get('x_coordinate', 0.0)
        #         y = item_info.get('y_coordinate', 0.0)

        #         msg = String()
        #         msg.data = f"{item_name},{x},{y}"
        #         self.shopping_list.publish(msg)
        #         self.get_logger().info(f'Published item: {item_name} at ({x}, {y})')
        

def main(args=None):
    rclpy.init(args=args)
    items_manager = ItemsManager()
    executor = MultiThreadedExecutor()
    executor.add_node(items_manager)
    executor.spin()
    items_manager.destroy_node()
    rclpy.shutdown()