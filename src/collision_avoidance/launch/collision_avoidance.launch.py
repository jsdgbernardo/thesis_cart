from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cart_collision_avoidance',
            executable='collision_avoidance_node',
            name='collision_avoidance_node',
            parameters=['config/collision_params.yaml']
        )
    ])
