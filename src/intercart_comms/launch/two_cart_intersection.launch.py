from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    cart1 = Node(
        package='intercart_comms',
        executable='intercart_comm_node',
        name='cart1_intercart_comm',
        parameters=[{
            'cart_id': 1,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'velocity_x': 0.5,
            'velocity_y': 0.0,
            'safety_radius': 1.2,
            'prediction_horizon': 3.0
        }],
        output='screen'
    )

    cart2 = Node(
        package='intercart_comms',
        executable='intercart_comm_node',
        name='cart2_intercart_comm',
        parameters=[{
            'cart_id': 2,
            'initial_x': 3.0,
            'initial_y': 3.0,
            'velocity_x': 0.0,
            'velocity_y': -0.5,
            'safety_radius': 1.2,
            'prediction_horizon': 3.0
        }],
        output='screen'
    )

    return LaunchDescription([
        cart1,
        cart2
    ])

# //test this does nothing