from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='price',
            executable='calculation',
            name='calculation',
            output='screen'
        ),
        Node(
            package='camera',
            executable='webcam',
            name='webcam',
            output='screen'
        ),
        Node(
            package='model',
            executable='yolo',
            name='yolo',
            output='screen'
        ),
    ])