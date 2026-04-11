#!/usr/bin/env python3

import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Path
from std_msgs.msg import String


def _get_launch_description() -> LaunchDescription:
    pkg_path = get_package_share_directory('path_planning')
    launch_path_file = os.path.join(pkg_path, 'launch', 'launch_path.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path_file),
            launch_arguments={'planner': 'metaheuristic'}.items(),
        ),
        OpaqueFunction(function=_start_test_thread),
    ])


def _start_test_thread(context, *args, **kwargs):
    thread = threading.Thread(target=_run_test_loop, daemon=True)
    thread.start()
    return []


def _kill_process(proc: subprocess.Popen, name: str) -> None:
    if proc is None:
        return
    if proc.poll() is not None:
        return
    try:
        # Try SIGINT first for graceful shutdown
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        try:
            # If still running, force kill
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            pass


class ShoppingListTester:
    def __init__(self, timeout: float = 60.0):
        self.timeout = timeout
        self.best_path_received = threading.Event()
        self.node = None
        self.publisher = None
        self._last_path = None
        self.planner_command = ['ros2', 'launch', 'path_planning', 'launch_planner.launch.py', 'astar:=true']

        # spatially distributed items
        # self.messages = [
        #     'chicken_leg',
        #     'chicken_leg,onion',
        #     'chicken_leg,onion,string_beans',
        #     'chicken_leg,onion,string_beans,piattos',
        #     'chicken_leg,onion,string_beans,piattos,sugar',
        #     'chicken_leg,onion,string_beans,piattos,milk,sugar',
        #     'chicken_leg,carrot,string_beans,onion,piattos,milk,sugar',
        #     'chicken_leg,carrot,string_beans,tuna,piattos,milk,sugar,onion',
        #     'chicken_leg,garlic,onion,tuna,string_beans,carrot,piattos,milk,sugar',
        #     'chicken_leg,chicken_quarter,garlic,onion,tuna,string_beans,carrot,piattos,milk,sugar',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,tuna,string_beans,carrot,piattos,milk,sugar',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,tuna,string_beans,carrot,vinegar,piattos,milk,sugar',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,tuna,string_beans,carrot,vinegar,piattos,milk,sugar,dishwashing_liquid',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,eggplant,carrot,string_beans,tuna,piattos,vinegar,dishwashing_liquid,sugar,milk',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,eggplant,carrot,string_beans,tuna,soy_sauce,piattos,vinegar,dishwashing_liquid,sugar,milk',
        # ]

        # # aisle items
        # self.messages = [
        #     'chicken_feet',
        #     'chicken_feet,chicken_leg',
        #     'chicken_feet,chicken_leg,chicken_quarter',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,eggplant',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,eggplant,carrot',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,eggplant,carrot,string_beans',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,eggplant,carrot,string_beans,tuna',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,eggplant,carrot,string_beans,tuna,pringles',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,eggplant,carrot,string_beans,tuna,pringles,piattos',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,eggplant,carrot,string_beans,tuna,pringles,piattos,noodles',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,eggplant,carrot,string_beans,tuna,pringles,piattos,noodles,dishwashing_liquid',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,eggplant,carrot,string_beans,tuna,pringles,piattos,noodles,dishwashing_liquid,sugar',
        #     'chicken_feet,chicken_leg,chicken_quarter,garlic,onion,eggplant,carrot,string_beans,tuna,pringles,piattos,noodles,dishwashing_liquid,sugar,milk',
        # ]

        # randomized items, 10
        self.messages = [
            [
                'pringles',
                'pringles,chicken_quarter',
                'pringles,chicken_quarter,dishwashing_liquid',
                'pringles,chicken_quarter,dishwashing_liquid,chicken_leg',
                'pringles,chicken_quarter,dishwashing_liquid,chicken_leg,carrot',
                'pringles,chicken_quarter,dishwashing_liquid,chicken_leg,carrot,onion',
                'pringles,chicken_quarter,dishwashing_liquid,chicken_leg,carrot,onion,sugar',
                'pringles,chicken_quarter,dishwashing_liquid,chicken_leg,carrot,onion,sugar,piattos',
                'pringles,chicken_quarter,dishwashing_liquid,chicken_leg,carrot,onion,sugar,piattos,noodles',
                'pringles,chicken_quarter,dishwashing_liquid,chicken_leg,carrot,onion,sugar,piattos,noodles,tuna',
                'pringles,chicken_quarter,dishwashing_liquid,chicken_leg,carrot,onion,sugar,piattos,noodles,tuna,chicken_feet',
                'pringles,chicken_quarter,dishwashing_liquid,chicken_leg,carrot,onion,sugar,piattos,noodles,tuna,chicken_feet,eggplant',
                'pringles,chicken_quarter,dishwashing_liquid,chicken_leg,carrot,onion,sugar,piattos,noodles,tuna,chicken_feet,eggplant,string_beans',
                'pringles,chicken_quarter,dishwashing_liquid,chicken_leg,carrot,onion,sugar,piattos,noodles,tuna,chicken_feet,eggplant,string_beans,garlic',
                'pringles,chicken_quarter,dishwashing_liquid,chicken_leg,carrot,onion,sugar,piattos,noodles,tuna,chicken_feet,eggplant,string_beans,garlic,milk'
            ],
            [
                'pringles',
                'pringles,tuna',
                'pringles,tuna,string_beans',
                'pringles,tuna,string_beans,eggplant',
                'pringles,tuna,string_beans,eggplant,chicken_feet',
                'pringles,tuna,string_beans,eggplant,chicken_feet,garlic',
                'pringles,tuna,string_beans,eggplant,chicken_feet,garlic,milk',
                'pringles,tuna,string_beans,eggplant,chicken_feet,garlic,milk,chicken_leg',
                'pringles,tuna,string_beans,eggplant,chicken_feet,garlic,milk,chicken_leg,onion',
                'pringles,tuna,string_beans,eggplant,chicken_feet,garlic,milk,chicken_leg,onion,carrot',
                'pringles,tuna,string_beans,eggplant,chicken_feet,garlic,milk,chicken_leg,onion,carrot,dishwashing_liquid',
                'pringles,tuna,string_beans,eggplant,chicken_feet,garlic,milk,chicken_leg,onion,carrot,dishwashing_liquid,sugar',
                'pringles,tuna,string_beans,eggplant,chicken_feet,garlic,milk,chicken_leg,onion,carrot,dishwashing_liquid,sugar,piattos',
                'pringles,tuna,string_beans,eggplant,chicken_feet,garlic,milk,chicken_leg,onion,carrot,dishwashing_liquid,sugar,piattos,noodles',
                'pringles,tuna,string_beans,eggplant,chicken_feet,garlic,milk,chicken_leg,onion,carrot,dishwashing_liquid,sugar,piattos,noodles,chicken_quarter'
            ],
            [
                'onion',
                'onion,piattos',
                'onion,piattos,garlic',
                'onion,piattos,garlic,string_beans',
                'onion,piattos,garlic,string_beans,dishwashing_liquid',
                'onion,piattos,garlic,string_beans,dishwashing_liquid,noodles',
                'onion,piattos,garlic,string_beans,dishwashing_liquid,noodles,chicken_leg',
                'onion,piattos,garlic,string_beans,dishwashing_liquid,noodles,chicken_leg,milk',
                'onion,piattos,garlic,string_beans,dishwashing_liquid,noodles,chicken_leg,milk,eggplant',
                'onion,piattos,garlic,string_beans,dishwashing_liquid,noodles,chicken_leg,milk,eggplant,tuna',
                'onion,piattos,garlic,string_beans,dishwashing_liquid,noodles,chicken_leg,milk,eggplant,tuna,sugar',
                'onion,piattos,garlic,string_beans,dishwashing_liquid,noodles,chicken_leg,milk,eggplant,tuna,sugar,chicken_quarter',
                'onion,piattos,garlic,string_beans,dishwashing_liquid,noodles,chicken_leg,milk,eggplant,tuna,sugar,chicken_quarter,pringles',
                'onion,piattos,garlic,string_beans,dishwashing_liquid,noodles,chicken_leg,milk,eggplant,tuna,sugar,chicken_quarter,pringles,carrot',
                'onion,piattos,garlic,string_beans,dishwashing_liquid,noodles,chicken_leg,milk,eggplant,tuna,sugar,chicken_quarter,pringles,carrot,chicken_feet'
            ],
            [
                'pringles',
                'pringles,eggplant',
                'pringles,eggplant,carrot',
                'pringles,eggplant,carrot,dishwashing_liquid',
                'pringles,eggplant,carrot,dishwashing_liquid,garlic',
                'pringles,eggplant,carrot,dishwashing_liquid,garlic,onion',
                'pringles,eggplant,carrot,dishwashing_liquid,garlic,onion,milk',
                'pringles,eggplant,carrot,dishwashing_liquid,garlic,onion,milk,noodles',
                'pringles,eggplant,carrot,dishwashing_liquid,garlic,onion,milk,noodles,chicken_leg',
                'pringles,eggplant,carrot,dishwashing_liquid,garlic,onion,milk,noodles,chicken_leg,piattos',
                'pringles,eggplant,carrot,dishwashing_liquid,garlic,onion,milk,noodles,chicken_leg,piattos,string_beans',
                'pringles,eggplant,carrot,dishwashing_liquid,garlic,onion,milk,noodles,chicken_leg,piattos,string_beans,tuna',
                'pringles,eggplant,carrot,dishwashing_liquid,garlic,onion,milk,noodles,chicken_leg,piattos,string_beans,tuna,chicken_quarter',
                'pringles,eggplant,carrot,dishwashing_liquid,garlic,onion,milk,noodles,chicken_leg,piattos,string_beans,tuna,chicken_quarter,sugar',
                'pringles,eggplant,carrot,dishwashing_liquid,garlic,onion,milk,noodles,chicken_leg,piattos,string_beans,tuna,chicken_quarter,sugar,chicken_feet'
            ],
            [
                'garlic',
                'garlic,string_beans',
                'garlic,string_beans,chicken_feet',
                'garlic,string_beans,chicken_feet,chicken_quarter',
                'garlic,string_beans,chicken_feet,chicken_quarter,dishwashing_liquid',
                'garlic,string_beans,chicken_feet,chicken_quarter,dishwashing_liquid,milk',
                'garlic,string_beans,chicken_feet,chicken_quarter,dishwashing_liquid,milk,piattos',
                'garlic,string_beans,chicken_feet,chicken_quarter,dishwashing_liquid,milk,piattos,carrot',
                'garlic,string_beans,chicken_feet,chicken_quarter,dishwashing_liquid,milk,piattos,carrot,noodles',
                'garlic,string_beans,chicken_feet,chicken_quarter,dishwashing_liquid,milk,piattos,carrot,noodles,chicken_leg',
                'garlic,string_beans,chicken_feet,chicken_quarter,dishwashing_liquid,milk,piattos,carrot,noodles,chicken_leg,pringles',
                'garlic,string_beans,chicken_feet,chicken_quarter,dishwashing_liquid,milk,piattos,carrot,noodles,chicken_leg,pringles,eggplant',
                'garlic,string_beans,chicken_feet,chicken_quarter,dishwashing_liquid,milk,piattos,carrot,noodles,chicken_leg,pringles,eggplant,onion',
                'garlic,string_beans,chicken_feet,chicken_quarter,dishwashing_liquid,milk,piattos,carrot,noodles,chicken_leg,pringles,eggplant,onion,sugar',
                'garlic,string_beans,chicken_feet,chicken_quarter,dishwashing_liquid,milk,piattos,carrot,noodles,chicken_leg,pringles,eggplant,onion,sugar,tuna'
            ],
            [
                'chicken_feet',
                'chicken_feet,eggplant',
                'chicken_feet,eggplant,garlic',
                'chicken_feet,eggplant,garlic,dishwashing_liquid',
                'chicken_feet,eggplant,garlic,dishwashing_liquid,tuna',
                'chicken_feet,eggplant,garlic,dishwashing_liquid,tuna,chicken_quarter',
                'chicken_feet,eggplant,garlic,dishwashing_liquid,tuna,chicken_quarter,milk',
                'chicken_feet,eggplant,garlic,dishwashing_liquid,tuna,chicken_quarter,milk,piattos',
                'chicken_feet,eggplant,garlic,dishwashing_liquid,tuna,chicken_quarter,milk,piattos,sugar',
                'chicken_feet,eggplant,garlic,dishwashing_liquid,tuna,chicken_quarter,milk,piattos,sugar,chicken_leg',
                'chicken_feet,eggplant,garlic,dishwashing_liquid,tuna,chicken_quarter,milk,piattos,sugar,chicken_leg,onion',
                'chicken_feet,eggplant,garlic,dishwashing_liquid,tuna,chicken_quarter,milk,piattos,sugar,chicken_leg,onion,string_beans',
                'chicken_feet,eggplant,garlic,dishwashing_liquid,tuna,chicken_quarter,milk,piattos,sugar,chicken_leg,onion,string_beans,carrot',
                'chicken_feet,eggplant,garlic,dishwashing_liquid,tuna,chicken_quarter,milk,piattos,sugar,chicken_leg,onion,string_beans,carrot,pringles',
                'chicken_feet,eggplant,garlic,dishwashing_liquid,tuna,chicken_quarter,milk,piattos,sugar,chicken_leg,onion,string_beans,carrot,pringles,noodles'
            ],
            [
                'tuna',
                'tuna,chicken_leg',
                'tuna,chicken_leg,garlic',
                'tuna,chicken_leg,garlic,noodles',
                'tuna,chicken_leg,garlic,noodles,eggplant',
                'tuna,chicken_leg,garlic,noodles,eggplant,onion',
                'tuna,chicken_leg,garlic,noodles,eggplant,onion,string_beans',
                'tuna,chicken_leg,garlic,noodles,eggplant,onion,string_beans,sugar',
                'tuna,chicken_leg,garlic,noodles,eggplant,onion,string_beans,sugar,milk',
                'tuna,chicken_leg,garlic,noodles,eggplant,onion,string_beans,sugar,milk,carrot',
                'tuna,chicken_leg,garlic,noodles,eggplant,onion,string_beans,sugar,milk,carrot,chicken_quarter',
                'tuna,chicken_leg,garlic,noodles,eggplant,onion,string_beans,sugar,milk,carrot,chicken_quarter,piattos',
                'tuna,chicken_leg,garlic,noodles,eggplant,onion,string_beans,sugar,milk,carrot,chicken_quarter,piattos,dishwashing_liquid',
                'tuna,chicken_leg,garlic,noodles,eggplant,onion,string_beans,sugar,milk,carrot,chicken_quarter,piattos,dishwashing_liquid,pringles',
                'tuna,chicken_leg,garlic,noodles,eggplant,onion,string_beans,sugar,milk,carrot,chicken_quarter,piattos,dishwashing_liquid,pringles,chicken_feet'
            ],
            [
                'string_beans',
                'string_beans,tuna',
                'string_beans,tuna,dishwashing_liquid',
                'string_beans,tuna,dishwashing_liquid,piattos',
                'string_beans,tuna,dishwashing_liquid,piattos,eggplant',
                'string_beans,tuna,dishwashing_liquid,piattos,eggplant,onion',
                'string_beans,tuna,dishwashing_liquid,piattos,eggplant,onion,chicken_quarter',
                'string_beans,tuna,dishwashing_liquid,piattos,eggplant,onion,chicken_quarter,milk',
                'string_beans,tuna,dishwashing_liquid,piattos,eggplant,onion,chicken_quarter,milk,carrot',
                'string_beans,tuna,dishwashing_liquid,piattos,eggplant,onion,chicken_quarter,milk,carrot,chicken_leg',
                'string_beans,tuna,dishwashing_liquid,piattos,eggplant,onion,chicken_quarter,milk,carrot,chicken_leg,pringles',
                'string_beans,tuna,dishwashing_liquid,piattos,eggplant,onion,chicken_quarter,milk,carrot,chicken_leg,pringles,chicken_feet',
                'string_beans,tuna,dishwashing_liquid,piattos,eggplant,onion,chicken_quarter,milk,carrot,chicken_leg,pringles,chicken_feet,sugar',
                'string_beans,tuna,dishwashing_liquid,piattos,eggplant,onion,chicken_quarter,milk,carrot,chicken_leg,pringles,chicken_feet,sugar,noodles',
                'string_beans,tuna,dishwashing_liquid,piattos,eggplant,onion,chicken_quarter,milk,carrot,chicken_leg,pringles,chicken_feet,sugar,noodles,garlic'
            ],
            [
                'tuna',
                'tuna,carrot',
                'tuna,carrot,chicken_leg',
                'tuna,carrot,chicken_leg,dishwashing_liquid',
                'tuna,carrot,chicken_leg,dishwashing_liquid,onion',
                'tuna,carrot,chicken_leg,dishwashing_liquid,onion,chicken_quarter',
                'tuna,carrot,chicken_leg,dishwashing_liquid,onion,chicken_quarter,pringles',
                'tuna,carrot,chicken_leg,dishwashing_liquid,onion,chicken_quarter,pringles,eggplant',
                'tuna,carrot,chicken_leg,dishwashing_liquid,onion,chicken_quarter,pringles,eggplant,garlic',
                'tuna,carrot,chicken_leg,dishwashing_liquid,onion,chicken_quarter,pringles,eggplant,garlic,string_beans',
                'tuna,carrot,chicken_leg,dishwashing_liquid,onion,chicken_quarter,pringles,eggplant,garlic,string_beans,chicken_feet',
                'tuna,carrot,chicken_leg,dishwashing_liquid,onion,chicken_quarter,pringles,eggplant,garlic,string_beans,chicken_feet,noodles',
                'tuna,carrot,chicken_leg,dishwashing_liquid,onion,chicken_quarter,pringles,eggplant,garlic,string_beans,chicken_feet,noodles,sugar',
                'tuna,carrot,chicken_leg,dishwashing_liquid,onion,chicken_quarter,pringles,eggplant,garlic,string_beans,chicken_feet,noodles,sugar,milk',
                'tuna,carrot,chicken_leg,dishwashing_liquid,onion,chicken_quarter,pringles,eggplant,garlic,string_beans,chicken_feet,noodles,sugar,milk,piattos'
            ],
            [
                'chicken_leg',
                'chicken_leg,pringles',
                'chicken_leg,pringles,dishwashing_liquid',
                'chicken_leg,pringles,dishwashing_liquid,eggplant',
                'chicken_leg,pringles,dishwashing_liquid,eggplant,sugar',
                'chicken_leg,pringles,dishwashing_liquid,eggplant,sugar,garlic',
                'chicken_leg,pringles,dishwashing_liquid,eggplant,sugar,garlic,piattos',
                'chicken_leg,pringles,dishwashing_liquid,eggplant,sugar,garlic,piattos,carrot',
                'chicken_leg,pringles,dishwashing_liquid,eggplant,sugar,garlic,piattos,carrot,onion',
                'chicken_leg,pringles,dishwashing_liquid,eggplant,sugar,garlic,piattos,carrot,onion,string_beans',
                'chicken_leg,pringles,dishwashing_liquid,eggplant,sugar,garlic,piattos,carrot,onion,string_beans,milk',
                'chicken_leg,pringles,dishwashing_liquid,eggplant,sugar,garlic,piattos,carrot,onion,string_beans,milk,chicken_quarter',
                'chicken_leg,pringles,dishwashing_liquid,eggplant,sugar,garlic,piattos,carrot,onion,string_beans,milk,chicken_quarter,chicken_feet',
                'chicken_leg,pringles,dishwashing_liquid,eggplant,sugar,garlic,piattos,carrot,onion,string_beans,milk,chicken_quarter,chicken_feet,noodles',
                'chicken_leg,pringles,dishwashing_liquid,eggplant,sugar,garlic,piattos,carrot,onion,string_beans,milk,chicken_quarter,chicken_feet,noodles,tuna'
            ]
        ]

        self.loops = len(self.messages)

    def best_path_callback(self, msg: Path) -> None:
        self._last_path = msg
        self.best_path_received.set()

    def publish_message(self, payload: str) -> None:
        if self.publisher is None:
            return
        msg = String()
        msg.data = payload
        self.publisher.publish(msg)

    def _wait_for_planner(self) -> None:
        self.node.get_logger().info('Waiting for planner node to be active...')
        while True:
            node_names = [name for name, ns in self.node.get_node_names_and_namespaces()]
            if any('planner' in name.lower() for name in node_names):
                time.sleep(6.5)  # Give it more time to fully initialize
                self.node.get_logger().info('Planner node detected and initialized.')
                break
            rclpy.spin_once(self.node, timeout_sec=1.0)
            time.sleep(0.1)

    def run(self) -> None:
        rclpy.init(args=None)
        self.node = rclpy.create_node('shopping_list_test_node')
        self.publisher = self.node.create_publisher(String, '/shopping_list', 10)
        self.node.create_subscription(Path, '/best_path', self.best_path_callback, 10)
        self.node.get_logger().info('Shopping list test node created.')

        # Launch the path system (launch_path.launch.py with planner='none') and keep it running
        path_command = ['ros2', 'launch', 'path_planning', 'launch_path.launch.py', 'planner:=metaheuristic']
        path_proc = subprocess.Popen(
            path_command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        self.node.get_logger().info(f'Launched path system with command: {" ".join(path_command)} (pid={path_proc.pid}).')

        try:
            # Allow the launched system to finish initializing before we begin.
            self.node.get_logger().info('Waiting for path system to initialize...')
            self._sleep_with_spin(8.0)

            for run in self.messages:

                for iteration, payload in enumerate(run, start=1):

                    self.node.get_logger().info(f'Iteration {iteration}/{self.loops}: Launching planner...')

                    # Launch the planner
                    planner_proc = subprocess.Popen(
                        self.planner_command,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        start_new_session=True,
                    )
                    self.node.get_logger().info(f'Launched planner with pid={planner_proc.pid}')

                    # Wait for the planner to be active
                    self._wait_for_planner()

                    # Send the message
                    self.publish_message(payload)
                    self.node.get_logger().info(f'Sent shopping list: {payload}')

                    # Wait for best_path to be received
                    self.best_path_received.clear()
                    self.node.get_logger().info('Waiting for /best_path...')
                    start_time = time.time()
                    while time.time() - start_time < self.timeout:
                        rclpy.spin_once(self.node, timeout_sec=0.25)
                        if self.best_path_received.is_set():
                            self.node.get_logger().info(f'Received /best_path for iteration {iteration}.')
                            break

                    if not self.best_path_received.is_set():
                        self.node.get_logger().error(f'No /best_path received within {self.timeout} seconds.')

                    # Terminate the planner
                    self.node.get_logger().info(f'Terminating planner for iteration {iteration}...')
                    _kill_process(planner_proc, " ".join(self.planner_command))
                    self.node.get_logger().info(f'Planner terminated for iteration {iteration}.')

            self.node.get_logger().info('All messages sent. Test complete.')

        finally:
            if path_proc is not None:
                _kill_process(path_proc, " ".join(path_command))
            self.node.destroy_node()
            rclpy.shutdown()

    def _sleep_with_spin(self, duration: float) -> None:
        deadline = time.time() + duration
        message = String()
        message.data = 'warmup_message'
        while time.time() < deadline:
            if self.publisher is not None:
                self.publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)


def _run_test_loop() -> None:
    try:
        tester = ShoppingListTester()
        tester.run()
    except Exception as exc:
        print(f'Error in test thread: {exc}', file=sys.stderr)


def generate_launch_description() -> LaunchDescription:
    return _get_launch_description()


if __name__ == '__main__':
    # Allow the test file to be run directly as a Python script.
    _run_test_loop()
