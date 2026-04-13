"""
full_system.launch.py
=====================
Launches the complete cobot system:
  - robot_state_publisher (URDF → TF2)
  - joint_state_publisher_gui (interactive sliders)
  - RViz2 (visualisation)
  - impedance_controller (3-layer cascaded control)
  - vision_node (object detection / simulation)

This is the full-system demo launch for Articulus Surgical and
Integra Robotics showcase. All nodes publish to their respective
topics simultaneously — the system is fully integrated.

Usage:
  ros2 launch cobot_description full_system.launch.py
  ros2 launch cobot_description full_system.launch.py gui:=false vision:=false
  ros2 launch cobot_description full_system.launch.py mode:=lab
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = FindPackageShare(package='cobot_description').find('cobot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'cobot.urdf.xacro')
    rviz_file = os.path.join(pkg_share, 'rviz', 'cobot_view.rviz')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_file]), value_type=str
        )
    }

    # ── Launch arguments ─────────────────────────────────────────────────────
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Enable joint_state_publisher_gui sliders')

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2')

    declare_vision = DeclareLaunchArgument(
        'vision', default_value='true',
        description='Launch vision perception node')

    declare_impedance = DeclareLaunchArgument(
        'impedance', default_value='true',
        description='Launch impedance controller')

    declare_mode = DeclareLaunchArgument(
        'mode', default_value='surgical',
        description='Impedance mode: surgical | lab | compliant')

    declare_camera = DeclareLaunchArgument(
        'use_camera', default_value='false',
        description='Use real USB camera (false = simulation)')

    # ── Core nodes ───────────────────────────────────────────────────────────

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # ── Impedance controller — starts after 2s to let robot_state_publisher init
    impedance_controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='cobot_description',
                executable='impedance_controller',
                name='impedance_controller',
                output='screen',
                parameters=[{
                    'mode': LaunchConfiguration('mode'),
                    'gravity_comp_enabled': True,
                    'vibration_damping_enabled': True,
                    'torque_limit_factor': 0.85,
                }],
                condition=IfCondition(LaunchConfiguration('impedance'))
            )
        ]
    )

    # ── Vision node — starts after 3s
    vision_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='cobot_description',
                executable='vision_node',
                name='vision_node',
                output='screen',
                parameters=[{
                    'use_camera': LaunchConfiguration('use_camera'),
                    'detect_mode': 'labware',
                    'sim_fps': 10.0,
                    'show_window': False,
                }],
                condition=IfCondition(LaunchConfiguration('vision'))
            )
        ]
    )

    # ── Log startup message ───────────────────────────────────────────────────
    startup_msg = LogInfo(msg=(
        '\n'
        '╔══════════════════════════════════════════════════════╗\n'
        '║    6-DOF Teleoperated Precision Cobot — Full System  ║\n'
        '╠══════════════════════════════════════════════════════╣\n'
        '║  robot_state_publisher  → /robot_description        ║\n'
        '║  joint_state_publisher  → /joint_states             ║\n'
        '║  impedance_controller   → /impedance_cmd            ║\n'
        '║  vision_node            → /vision/detection         ║\n'
        '║  rviz2                  → 3D visualisation          ║\n'
        '╠══════════════════════════════════════════════════════╣\n'
        '║  To teleoperate (new terminal):                     ║\n'
        '║  ros2 run cobot_description teleop_node             ║\n'
        '╚══════════════════════════════════════════════════════╝\n'
    ))

    return LaunchDescription([
        declare_gui,
        declare_rviz,
        declare_vision,
        declare_impedance,
        declare_mode,
        declare_camera,
        startup_msg,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz2,
        impedance_controller,
        vision_node,
    ])
