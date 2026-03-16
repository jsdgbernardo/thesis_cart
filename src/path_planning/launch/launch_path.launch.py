import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Get path
    pkg_path = get_package_share_directory('path_planning')

    # Launch arguments
    planner = LaunchConfiguration('planner')
    planner_arg = DeclareLaunchArgument(
        'planner',
        default_value='brute_force',
        description='Apprioach to use (brute_force, heuristic, metaheuristic)'
    )

    # Items manager node
    items_manager = Node(
        package='path_planning',
        executable='items_manager_node',
        name='items_manager_node',
        output='screen'
    )

    # Rosbridge launch file
    rosbridge_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'
        ],
        output='screen'
    )

    # Brute force approach
    brute_force = Node(
        package='path_planning',
        executable='brute_force_node',
        name='brute_force_node',
        output='screen',
        condition=IfCondition(PythonExpression(['"', planner, '" == "brute_force"']))
    )

    # Heuristic approach
    heuristic = Node(
        package='path_planning',
        executable='heuristic_node',
        name='heuristic_node',
        output='screen',
        condition=IfCondition(PythonExpression(['"', planner, '" == "heuristic"']))
    )

    # Meta-heuristic approach
    meta_heuristic = Node(
        package='path_planning',
        executable='meta_heuristic_node',
        name='meta_heuristic_node',
        output='screen',
        condition=IfCondition(PythonExpression(['"', planner, '" == "metaheuristic"']))
    )

    return LaunchDescription([
        planner_arg,
        rosbridge_launch,
        items_manager,
        brute_force,
        heuristic,
        meta_heuristic
    ])