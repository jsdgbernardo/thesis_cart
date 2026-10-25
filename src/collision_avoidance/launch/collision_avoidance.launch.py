
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('collision_avoidance'),
        'config',
        'collision_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='collision_avoidance',
            executable='collision_avoidance_node',
            name='collision_avoidance_node',
            output='screen',
            parameters=[config_file]
        )
    ])
