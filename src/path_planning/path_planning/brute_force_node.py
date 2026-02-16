import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory

from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, Pose, PoseArray, Quaternion
from tf_transformations import quaternion_from_euler
from std_msgs.msg import String, Float32
from itertools import permutations

import os, json
from .items_manager import ItemsManager

class ItemCoordinates:
    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y

class BruteForceNode(Node):
    def __init__(self):
        super().__init__('brute_force_node')

        self.items = [] # Will be populated from the item manager's published shopping list
        self.json_items = self.load_items() # Load item locations from JSON for reference

        # Initialize action client for path planning
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # Subcribe to shopping list from the item manager
        self.shopping_list = self.create_subscription(
            String,
            'shopping_list',
            self.shopping_list_callback,
            10
        )

        # Subscribe to AMCL pose for current robot position
        self.current_pose = None
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_callback,
            10
        )

        # Publishes the best path found
        self.path_publisher = self.create_publisher(Path, 'best_path', 10)
        self.order_publisher = self.create_publisher(String, 'goal_order', 10)
        
        self.get_logger().info(f'BruteForceNode initialized.')

    # Load items and their locations from JSON file
    def load_items(self):
        price_path = get_package_share_directory('price')
        json_path = os.path.join(price_path,'items','grocery_items.json')
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)

            return data
        
        except Exception as e:
            self.get_logger().error(f'Error loading items from JSON: {e}')
            return []


    # Calls for shopping list updates from the item manager
    # Triggers path planning algorithm when new shopping list is received
    def shopping_list_callback(self, msg):
        
        item_name = msg.data.split(',')
        self.items = [] # Clear previous items

        for name in item_name:
            if name in self.json_items:
                x = self.json_items[name]['x_coordinate']
                y = self.json_items[name]['y_coordinate']
                self.items.append(ItemCoordinates(name, x, y))
                self.get_logger().info(f'Received shopping list item: {item_name} at ({x}, {y})')
            else:
                self.get_logger().warn(f'Item {name} not found in JSON database.')

        self.get_logger().info(f'Recomputing path.')
        # Trigger planning immediately with the updated items
        if self.current_pose is not None and len(self.items) > 0:
            self.computer_and_publish_path(self.items)

    # Update current robot pose from AMCL
    def amcl_callback(self, msg):
        self.current_pose = msg.pose.pose

    # Compute Brute Force TSP + Dijkstra Path for the given shopping list items
    def computer_and_publish_path(self, items):

        # Convert current pose to PoseStamped
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.pose = self.current_pose
        
        # Convert items to PoseStamped goals
        goal_poses = [self.item_to_pose_stamped(it) for it in items]

        # Compute TSP + Dijkstra
        best_path = None
        best_cost = float('inf')
        best_order = None

        if not goal_poses:
            self.get_logger().warn('No goal poses to plan.')
            return

        # Try all permutations of goal orders (Brute Force TSP)
        for order in permutations(range(len(goal_poses))):
            ordered_goals = [goal_poses[i] for i in order]
            path, cost = self.compute_full_path(start_pose, ordered_goals)
            if path is not None and cost < best_cost:
                best_path = path
                best_cost = cost
                best_order = [items[i].name for i in order]

        if best_path:
            # publish path and order
            self.path_publisher.publish(best_path)
            order_msg = String()
            order_msg.data = json.dumps({'order': best_order, 'cost': best_cost})
            self.order_publisher.publish(order_msg)
            self.get_logger().info(f'Published best path. Order: {best_order}, Cost: {best_cost:.2f}')
        else:
            self.get_logger().error('No valid path found for selected items.')

    # Concatenate path segments between start and each successive goal
    def compute_full_path(self, start_pose, goals):
        total_path = Path()
        total_cost = 0.0
        poses = [start_pose] + list(goals)
        for i in range(len(poses) - 1):
            path, cost = self.compute_path_segment(poses[i], poses[i + 1])
            if path is None:
                return None, float('inf')
            # extend poses (avoid duplicating overlapping waypoints if desired)
            total_path.poses.extend(path.poses)
            total_cost += cost
        return total_path, total_cost

    # Call nav2 ComputePathToPose action to get path between two PoseStamped
    # Use nav2_params config file for Dijkstra settings
    def compute_path_segment(self, start, goal):
        goal_msg = ComputePathToPose.Goal()
        # nav2 expects PoseStamped for start and goal
        goal_msg.start = start
        goal_msg.goal = goal

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('compute_path_to_pose action server not available.')
            return None, float('inf')

        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done():
            result = future.result()
            if result and result.result and result.result.path and len(result.result.path.poses) > 0:
                path = result.result.path
                cost = self.path_cost(path)
                return path, cost

        return None, float('inf')

    # Compute simple Euclidean cost along a nav_msgs/Path
    # Uses path waypoints to calculate total distance traveled
    def path_cost(self, path):
        poses = path.poses
        cost = 0.0
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position
            dx = p1.x - p2.x
            dy = p1.y - p2.y
            dz = p1.z - p2.z
            cost += (dx * dx + dy * dy + dz * dz) ** 0.5
        return cost

    # Convert item coordinates to a pose stamped goal for path planning
    def item_to_pose_stamped(self, item: ItemCoordinates, frame_id='map', yaw=0.0):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id

        pose.pose.position.x = item.x
        pose.pose.position.y = item.y
        pose.pose.position.z = 0.0

        # Convert yaw to quaternion (facing forward)
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose

def main(args=None):
    rclpy.init(args=args)
    node = BruteForceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()