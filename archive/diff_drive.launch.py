# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable, TextSubstitution
# subprocess not needed when using Command/FindExecutable
import shutil


from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('chassis')
    # Use the local package's `worlds` directory for the example world
    pkg_project_gazebo = get_package_share_directory('chassis')
    pkg_project_description = get_package_share_directory('chassis')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the URDF (process xacro if present) from the description package
    urdf_xacro = os.path.join(pkg_project_description, 'description', 'chassis.urdf.xacro')
    urdf_file = os.path.join(pkg_project_description, 'description', 'chassis.urdf')

    if os.path.exists(urdf_xacro):
        # Use the Command substitution to run xacro at launch time
        robot_description = Command([FindExecutable(name='xacro'), ' ', urdf_xacro])
    elif os.path.exists(urdf_file):
        # static URDF file
        with open(urdf_file, 'r') as infp:
            robot_description = infp.read()
    else:
        raise FileNotFoundError("Could not find 'chassis.urdf.xacro' or 'chassis.urdf' in package 'chassis'")


    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'empty.sdf'
        ])}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Spawn the URDF into the simulator after it starts
    model_name = 'diff_drive'
    # Spawn a bit above the ground; default is 0.07 m (configurable via spawn_z)
    spawn_cmd = ['bash', '-lc', [f"xacro {urdf_xacro} > /tmp/{model_name}.urdf && ros2 run ros_gz_sim create --file /tmp/{model_name}.urdf --name {model_name} --type urdf --pose \"0 0 ", LaunchConfiguration('spawn_z'), " 0 0 0\""]]
    # Only spawn when requested
    spawn = TimerAction(period=3.0, actions=[ExecuteProcess(cmd=spawn_cmd, output='screen', condition=IfCondition(LaunchConfiguration('spawn')))])

    # Start local kinematic controller that listens to /diff_drive/cmd_vel and publishes /joint_states and odom
    diff_drive_controller = Node(
        package='chassis',
        executable='diff_drive_controller',
        name='diff_drive_controller',
        output='screen',
        parameters=[
            {'wheel_radius': 0.0325},
            {'wheel_separation': 0.18288},
        ]
    )

    # Teleop process: prefer opening an interactive terminal so teleop_twist_keyboard has a TTY
    if shutil.which('gnome-terminal'):
        teleop_process = ExecuteProcess(cmd=['gnome-terminal', '--', 'bash', '-lc', [LaunchConfiguration('teleop_cmd'), TextSubstitution(text='; exec bash')]], output='screen', condition=IfCondition(LaunchConfiguration('teleop')))
    elif shutil.which('konsole'):
        teleop_process = ExecuteProcess(cmd=['konsole', '-e', 'bash', '-lc', [LaunchConfiguration('teleop_cmd'), TextSubstitution(text='; exec bash')]], output='screen', condition=IfCondition(LaunchConfiguration('teleop')))
    elif shutil.which('xterm'):
        teleop_process = ExecuteProcess(cmd=['xterm', '-e', 'bash', '-lc', [LaunchConfiguration('teleop_cmd'), TextSubstitution(text='; exec bash')]], output='screen', condition=IfCondition(LaunchConfiguration('teleop')))
    else:
        # Fallback: try to allocate a pseudo-TTY so teleop_twist_keyboard can read stdin
        teleop_process = ExecuteProcess(cmd=[LaunchConfiguration('teleop_cmd')], shell=True, output='screen', use_pty=True, condition=IfCondition(LaunchConfiguration('teleop')))


    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('spawn', default_value='true',
                              description='Spawn URDF into simulator.'),
        DeclareLaunchArgument('spawn_z', default_value='0.10',
                              description='Spawn height (meters) for the model.'),
        DeclareLaunchArgument('teleop', default_value='false',
                              description='Run teleop_twist_keyboard to control the robot.'),
        DeclareLaunchArgument('teleop_cmd', default_value='ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive/cmd_vel',
                              description='Command to run the teleop (can include terminal wrapper).'),
        bridge,
        robot_state_publisher,
        diff_drive_controller,
        rviz,
        spawn,
        # Teleop process inserted above
        teleop_process,
    ])