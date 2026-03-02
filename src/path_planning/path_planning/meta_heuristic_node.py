import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, Pose, PoseArray, Quaternion
from tf_transformations import quaternion_from_euler
from std_msgs.msg import String, Float32
from itertools import permutations
from time import sleep
from action_msgs.msg import GoalStatus
import threading

import random
import traceback

import os, json
from path_planning.items_manager import ItemsManager

class ItemCoordinates:
    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y

class MetaHeuristicNode(Node):
    def __init__(self):
        super().__init__('meta_heuristic_node')

        # allow overriding action name via ROS parameter
        self.declare_parameter('action_name', 'compute_path_to_pose')
        self.action_name = self.get_parameter('action_name').get_parameter_value().string_value

        self.items = [] # Will be populated from the item manager's published shopping list
        self.json_items = self.load_items() # Load item locations from JSON for reference

        # Initialize action client for path planning (single-segment ComputePathToPose)
        self.action_client = ActionClient(self, ComputePathToPose, self.action_name)
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to shopping list from the item manager
        self.shopping_list_sub = self.create_subscription(
            String,
            'shopping_list',
            self.shopping_list_callback,
            qos_profile
        )
        self.get_logger().info('Created subscription to /shopping_list')

        # Subscribe to AMCL pose for current robot position
        self.current_pose = None
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_callback,
            10
        )
        self.get_logger().info('Created subscription to /amcl_pose')

        # Publishes the best path found (use transient local so rviz can see it after startup)
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(Path, 'best_path', path_qos)
        self.order_publisher = self.create_publisher(String, 'goal_order', qos_profile)
        
        self.get_logger().info(f'MetaHeuristicNode initialized.')

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
        try:
            self.get_logger().info(f'shopping_list_callback triggered with data: {msg.data}')
            
            item_name = msg.data.split(',')
            self.items = [] # Clear previous items

            for name in item_name:
                name = name.strip()  # Remove leading/trailing whitespace
                self.get_logger().info(f'Processing item: "{name}"')
                if name in self.json_items:
                    x = self.json_items[name]['x_coordinate']
                    y = self.json_items[name]['y_coordinate']
                    self.items.append(ItemCoordinates(name, x, y))
                    self.get_logger().info(f'Received shopping list item: {name} at ({x}, {y})')
                else:
                    self.get_logger().warn(f'Item {name} not found in JSON database. Available: {list(self.json_items.keys())}')

            # Trigger planning in background thread to avoid blocking the callback
            if self.current_pose is not None and len(self.items) > 0:
                self.get_logger().info(f'Starting path recomputation for {len(self.items)} items.')
                # Run planning in a separate thread to avoid blocking callbacks
                planning_thread = threading.Thread(target=self.computer_and_publish_path, args=(self.items,))
                planning_thread.daemon = True
                planning_thread.start()
            else:
                if self.current_pose is None:
                    self.get_logger().warn('Current pose is None, cannot plan')
                if len(self.items) == 0:
                    self.get_logger().warn('No valid items to plan for')
        except Exception as e:
            self.get_logger().error(f'Exception in shopping_list_callback: {e}\n{traceback.format_exc()}')

    # Update current robot pose from AMCL
    def amcl_callback(self, msg):
        self.current_pose = msg.pose.pose

    # Compute Genetic Algorithm + A* Path for the given shopping list items
    def computer_and_publish_path(self, items):
        try:
            # Convert current pose to PoseStamped
            start_pose = PoseStamped()
            start_pose.header.frame_id = 'map'
            start_pose.header.stamp = self.get_clock().now().to_msg()
            start_pose.pose = self.current_pose
            
            # Convert items to PoseStamped goals
            goal_poses = [self.item_to_pose_stamped(it) for it in items]
            n = len(goal_poses)

            if not goal_poses:
                self.get_logger().warn('No goal poses to plan.')
                return
            
            if n < 2:
                self.get_logger().warn("Not enough goals for GA. Using direct path.")
                # handle single goal directly
                seg_path, seg_cost = self.compute_path_segment(start_pose, goal_poses[0])
                if seg_path:
                    self.path_publisher.publish(seg_path)
                return

            self.get_logger().info(f'Computing GA path for {n} goal poses. This may take some time...')

            # Pairwase A* costs cache
            cost_cache = {}

            def get_segment_cost(pose_a, pose_b):
                key = (id(pose_a), id(pose_b))
                if key not in cost_cache:
                    path, cost = self.compute_path_segment(pose_a, pose_b)
                    if path is None:
                        return float('inf'), None
                    cost_cache[key] = (cost, path)
                return cost_cache[key]
            
            def fitness(order):
                total_cost = 0.0
                current_pose = start_pose
                for idx in order:
                    cost, _ = get_segment_cost(current_pose, goal_poses[idx])
                    if cost == float('inf'):
                        return float('inf')  # Invalid path segment
                    total_cost += cost
                    current_pose = goal_poses[idx]
                return total_cost
            
            # GA parameters
            population_size = 30
            generations = 40
            mutation_rate = 0.2

            # initialize population
            population = []
            base = list(range(n))
            for _ in range(population_size):
                candidate = base.copy()
                random.shuffle(candidate)
                population.append(candidate)

            # GA loop
            for _ in range(generations):
                population.sort(key=lambda x: fitness(x))
                next_generation = population[:5] # elitism: keep top 5

                while (len(next_generation) < population_size):
                    parent1 = random.choice(population[:15]) # select from top 15
                    parent2 = random.choice(population[:15])

                    # ordered crossover
                    start, end = sorted(random.sample(range(n), 2))
                    child = [None] * n
                    
                    for i in range(start, end):
                        child[i] = parent1[i]

                    child_index = end
                    for i in range(n):
                        gene = parent2[(end + i) % n]
                        if gene not in child:
                            child[child_index % n] = gene
                            child_index += 1
                    
                    # mutation
                    if random.random() < mutation_rate:
                        i, j = random.sample(range(n), 2)
                        child[i], child[j] = child[j], child[i]

                    next_generation.append(child)

                population = next_generation

            # best solution
            best_order_indices = min(population, key=lambda x: fitness(x))
            best_cost = fitness(best_order_indices)

            # build path
            best_path = Path()
            best_path.header.frame_id = 'map'
            best_path.header.stamp = self.get_clock().now().to_msg()

            current_pose = start_pose

            # concatenate path segments from the best order
            for idx in best_order_indices:
                cost, segment_path = get_segment_cost(current_pose, goal_poses[idx])
                if segment_path is None:
                    self.get_logger().error(f'Failed to compute path segment from {current_pose} to {goal_poses[idx]}')
                    best_path = None
                    break

                if best_path.poses and segment_path.poses:
                    best_path.poses.extend(segment_path.poses[1:])  # avoid duplicating the connecting pose
                else:
                    best_path.poses.extend(segment_path.poses)

                current_pose = goal_poses[idx]

            best_order = [items[i].name for i in best_order_indices]

            if best_path:
                self.get_logger().info(f'GA cost: {best_cost:.2f} for order: {best_order}')
            else:
                self.get_logger().error('GA planner failed to produce a valid path.')

            if best_path:
                # publish path and order
                self.path_publisher.publish(best_path)
                order_msg = String()
                order_msg.data = json.dumps({'order': best_order, 'cost': best_cost})
                self.order_publisher.publish(order_msg)
                self.get_logger().info(f'Published best path. Order: {best_order}, Cost: {best_cost:.2f}')
            else:
                self.get_logger().error('No valid path found for selected items.')
        except Exception as e:
            self.get_logger().error(f'Exception in computer_and_publish_path: {e}\n{traceback.format_exc()}')

    # Concatenate path segments between start and each successive goal
    def compute_full_path(self, start_pose, goals):
        total_path = Path()
        total_path.header.frame_id = 'map'
        total_path.header.stamp = self.get_clock().now().to_msg()
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
    # Use nav2_astar config file for A* settings
    def compute_path_segment(self, start: PoseStamped, goal: PoseStamped):
        self.get_logger().info(
            f'Computing path segment from ({start.pose.position.x:.2f}, {start.pose.position.y:.2f}) to ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
        )

        # wait for server with a few retries to handle startup timing
        if not self.wait_for_action_server(timeout_sec=5.0, retries=3):
            self.get_logger().error(f'ComputePathToPose action server not available (action_name={self.action_name}).')
            return None, float('inf')

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = ''
        goal_msg.use_start = True

        # Create a future to wait for completion
        import concurrent.futures
        result_ready = concurrent.futures.Future()

        def goal_response_callback(future):
            try:
                goal_handle = future.result()
                self.get_logger().info(f'Goal response received, accepted={goal_handle.accepted}')
                if not goal_handle.accepted:
                    self.get_logger().error('ComputePathToPose goal rejected')
                    result_ready.set_result((None, float('inf')))
                    return

                def result_callback(result_future):
                    try:
                        result_msg = result_future.result()
                        self.get_logger().info(f'Result callback triggered, result_msg={result_msg is not None}')
                        if result_msg is None:
                            self.get_logger().error('ComputePathToPose get_result returned None')
                            result_ready.set_result((None, float('inf')))
                            return

                        result = None
                        try:
                            result = result_msg.result
                        except Exception as e:
                            self.get_logger().error(f'Failed to extract result: {e}')
                            result = None

                        if result and result.path and len(result.path.poses) > 0:
                            path = result.path
                            cost = self.path_cost(path)
                            self.get_logger().info(f'Path result ready: {len(path.poses)} poses, cost={cost}')
                            result_ready.set_result((path, cost))
                        else:
                            self.get_logger().error(f'ComputePathToPose returned empty path. result={result}, has_path={result and hasattr(result, "path")}')
                            result_ready.set_result((None, float('inf')))
                    except Exception as e:
                        self.get_logger().error(f'Exception in result_callback: {e}\n{traceback.format_exc()}')
                        result_ready.set_result((None, float('inf')))

                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(result_callback)
            except Exception as e:
                self.get_logger().error(f'Exception in goal_response_callback: {e}\n{traceback.format_exc()}')
                result_ready.set_result((None, float('inf')))

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(goal_response_callback)

        # Wait for result with timeout (non-blocking on executor)
        try:
            path, cost = result_ready.result(timeout=20.0)
            return path, cost
        except concurrent.futures.TimeoutError:
            self.get_logger().error('ComputePathToPose action timed out')
            return None, float('inf')
        except Exception as e:
            self.get_logger().error(f'Exception waiting for path segment: {e}')
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

    def wait_for_action_server(self, timeout_sec: float = 5.0, retries: int = 3) -> bool:
        """Wait for the configured action server with retries.

        Returns True if available, False otherwise.
        """
        for attempt in range(1, retries + 1):
            self.get_logger().info(f'Waiting for action server "{self.action_name}" (attempt {attempt}/{retries})...')
            if self.action_client.wait_for_server(timeout_sec=timeout_sec):
                return True
            sleep(0.1)
        return False

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
    node = MetaHeuristicNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()