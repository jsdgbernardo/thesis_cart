import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory

from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, Pose, PoseArray, Quaternion
from tf_transformations import quaternion_from_euler
from std_msgs.msg import String, Float32
from itertools import permutations

import os, json
import heapq
import math
from .items_manager import ItemsManager

class ItemCoordinates:
    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y

class BruteForceNode(Node):
    def __init__(self):
        super().__init__('brute_force_node_filtered')

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
        # Subscribe to map (occupancy grid) to run Dijkstra on costmap
        self.map = None
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

        # Publishes the best path found
        self.path_publisher = self.create_publisher(Path, 'best_path', 10)
        self.order_publisher = self.create_publisher(String, 'goal_order', 10)
        
        self.get_logger().info(f'BruteForceNode (filtered) initialized.')

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
        # Use local occupancy grid and Dijkstra on grid graph
        if self.map is None:
            self.get_logger().warn('No map received yet; cannot run grid Dijkstra')
            return None, float('inf')

        # convert world start/goal to map indices
        sx, sy = self.world_to_map(start.pose.position.x, start.pose.position.y)
        gx, gy = self.world_to_map(goal.pose.position.x, goal.pose.position.y)

        if not self.in_bounds(sx, sy) or not self.in_bounds(gx, gy):
            return None, float('inf')

        # check occupancy (treat >50 as obstacle)
        if self.is_occupied(sx, sy) or self.is_occupied(gx, gy):
            return None, float('inf')

        came_from, dist = self.dijkstra((sx, sy), (gx, gy))
        if came_from is None:
            return None, float('inf')

        # reconstruct path (list of PoseStamped)
        path = Path()
        path.header.frame_id = self.map.header.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        node = (gx, gy)
        cells = []
        while node != (sx, sy):
            cells.append(node)
            node = came_from.get(node)
            if node is None:
                return None, float('inf')
        cells.append((sx, sy))
        cells.reverse()

        for (mx, my) in cells:
            px, py = self.map_to_world(mx, my)
            ps = PoseStamped()
            ps.header.frame_id = self.map.header.frame_id
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = px
            ps.pose.position.y = py
            ps.pose.position.z = 0.0
            # neutral orientation
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        cost = self.path_cost(path)
        return path, cost

    # Compute path length based on 2D distance and filter near-duplicate points
    def path_cost(self, path):
        poses = path.poses
        cost = 0.0
        prev = None
        for p in poses:
            pos = p.pose.position
            if prev is not None:
                dx = pos.x - prev.x
                dy = pos.y - prev.y
                dist = math.hypot(dx, dy)
                # ignore extremely small movements caused by noise
                if dist > 1e-4:
                    cost += dist
            prev = pos
        return cost

    # Map callback: cache occupancy grid
    def map_callback(self, msg: OccupancyGrid):
        self.map = msg

    def world_to_map(self, x, y):
        origin = self.map.info.origin.position
        res = self.map.info.resolution
        mx = int((x - origin.x) / res)
        my = int((y - origin.y) / res)
        return mx, my

    def map_to_world(self, mx, my):
        origin = self.map.info.origin.position
        res = self.map.info.resolution
        x = origin.x + (mx + 0.5) * res
        y = origin.y + (my + 0.5) * res
        return x, y

    def in_bounds(self, mx, my):
        w = self.map.info.width
        h = self.map.info.height
        return 0 <= mx < w and 0 <= my < h

    def is_occupied(self, mx, my):
        idx = my * self.map.info.width + mx
        val = self.map.data[idx]
        return val > 50

    def neighbors(self, node):
        x, y = node
        nbrs = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = x + dx
                ny = y + dy
                if not self.in_bounds(nx, ny):
                    continue
                if self.is_occupied(nx, ny):
                    continue
                cost = math.hypot(dx, dy)
                nbrs.append(((nx, ny), cost))
        return nbrs

    def dijkstra(self, start, goal):
        # classical Dijkstra on grid with heap
        frontier = []
        heapq.heappush(frontier, (0.0, start))
        came_from = {start: None}
        cost_so_far = {start: 0.0}

        while frontier:
            cur_cost, cur = heapq.heappop(frontier)
            if cur == goal:
                return came_from, cost_so_far[cur]
            for (n, w) in self.neighbors(cur):
                new_cost = cost_so_far[cur] + w
                if n not in cost_so_far or new_cost < cost_so_far[n]:
                    cost_so_far[n] = new_cost
                    came_from[n] = cur
                    heapq.heappush(frontier, (new_cost, n))

        return None, float('inf')

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
