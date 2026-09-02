from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_dir = get_package_share_directory(
        'intercart_comms'
    )

    config_file = os.path.join(
        package_dir,
        'config',
        'comms_params.yaml'
    )

    return LaunchDescription([

        Node(
            package='intercart_comms',
            executable='intercart_comm_node',
            name='intercart_comm_node',
            parameters=[config_file],
            output='screen'
        )

    ])