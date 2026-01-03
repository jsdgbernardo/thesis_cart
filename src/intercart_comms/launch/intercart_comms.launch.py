from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cart_intercart_comm',
            executable='intercart_comm_node',
            name='intercart_comm_node',
            parameters=['config/comm_params.yaml']
        )
    ])
