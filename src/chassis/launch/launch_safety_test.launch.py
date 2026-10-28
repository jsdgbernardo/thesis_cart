
import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    chassis_path = get_package_share_directory('chassis')
    collision_path = get_package_share_directory(
        'collision_avoidance'
    )

    twist_mux_config = os.path.join(
        chassis_path,
        'config',
        'twist_mux.yaml'
    )

    collision_config = os.path.join(
        collision_path,
        'config',
        'collision_params.yaml'
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config],
        remappings=[
            ('/cmd_vel_out', '/cmd_vel_requested')
        ]
    )

    collision_avoidance = Node(
        package='collision_avoidance',
        executable='collision_avoidance_node',
        name='collision_avoidance_node',
        output='screen',
        parameters=[collision_config],
        remappings=[
            ('/cmd_vel_manual', '/cmd_vel_requested'),
            (
                '/cmd_vel_collision',
                '/diff_drive/cmd_vel_unstamped'
            )
        ]
    )

    return LaunchDescription([
        twist_mux,
        collision_avoidance
    ])
