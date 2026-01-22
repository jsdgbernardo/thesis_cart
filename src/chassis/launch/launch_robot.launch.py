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

    # Lidar
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_path, 'launch', 'rplidar.launch.py'
        )])
    )

    # Controller params file
    controller_params_file = PathJoinSubstitution(
        [FindPackageShare('chassis'),
        'config',
        'my_controllers.yaml']
    )

    # ROS2 Control node spawners
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive",
            '--param-file',
            controller_params_file
            ],
        remappings=[( "/odom","/diff_drive/odom")],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    return LaunchDescription([
        robot_state_publisher,
        lidar,
        joint_broad_spawner,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_broad_spawner,
                on_exit=[diff_drive_spawner],
            )
        ),
    ])