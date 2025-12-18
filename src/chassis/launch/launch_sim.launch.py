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

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'])]
        ),
        launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    # bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     parameters=[{
    #         'config_file': os.path.join(pkg_path, 'config', 'ros_gz_bridge.yaml'),
    #         'qos_overrides./tf_static.publisher.durability': 'transient_local',
    #     }],
    #     output='screen'
    # )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'chassis',
        ],
        output='screen'
    )

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
    rviz = Node(
       package='rviz2',
       executable='rviz2',
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
        gz_sim,
        bridge,
        spawn_entity,
        robot_state_publisher,
        rviz,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_broad_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_broad_spawner,
                on_exit=[diff_drive_spawner],
            )
        ),
    ])