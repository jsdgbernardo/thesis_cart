import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    # Get paths
    pkg_path = get_package_share_directory('chassis')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load description file
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', 
        PathJoinSubstitution(
            [FindPackageShare('chassis'),
             'description',
            'chassis.urdf.xacro']
        )
    ])

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description}
        ]
    )

        # Visualize in RViz
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       condition=IfCondition(LaunchConfiguration('rviz')),
       output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher,
        rviz,
    ])