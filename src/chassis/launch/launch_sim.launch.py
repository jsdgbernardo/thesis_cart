import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch.conditions import IfCondition
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Get paths
    pkg_path = get_package_share_directory('chassis')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load description file
    xacro_file = os.path.join(pkg_path, 'description', 'chassis.urdf.xacro')
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', xacro_file
    ])

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_path,
            'worlds',
            'empty.sdf'
        ])}.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_path, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
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

    # # Controller params file
    # controller_params_file = os.path.join(
    #     pkg_path, 'config', 'my_controllers.yaml'
    # )

    # # ROS2 Control node
    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[{'robot_description': robot_description},
    #                 controller_params_file],
    #     output="screen",
    # )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive"],
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
        # controller_manager,
        diff_drive_spawner,
        joint_broad_spawner,
    ])