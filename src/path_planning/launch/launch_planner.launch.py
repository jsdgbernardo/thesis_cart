import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Get path
    launch_path = get_package_share_directory('chassis')

    # Launch arguments
    astar = LaunchConfiguration('astar')
    astar_arg = DeclareLaunchArgument(
        'astar',
        default_value='false',
        description='Use A* planner'
    )

    # Localization launch file
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_path, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map': os.path.join('map', 'processed.yaml')
        }.items()
    )

    # Navigation launch file for dijkstra
    navigation_dijkstra = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_path, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map_subscribe_transient_local': 'true'
        }.items(),
        condition=UnlessCondition(astar)
    )

    # Navigation launch file for A*
    navigation_astar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_path, 'launch', 'navigation_astar.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map_subscribe_transient_local': 'true'
        }.items(),
        condition=IfCondition(astar)
    )

    wait_for_initialpose = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '/initialpose', '--once'],
        output='screen'
    )

    start_navigation_after_pose = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_initialpose,
            on_exit=[
                navigation_dijkstra,
                navigation_astar
            ]
        )
    )

    return LaunchDescription([
        astar_arg,
        localization,
        wait_for_initialpose,
        start_navigation_after_pose
    ])