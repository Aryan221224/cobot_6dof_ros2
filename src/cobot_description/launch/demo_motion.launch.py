"""
Launch file: demo_motion.launch.py
Launches the cobot in RViz and runs an automated joint trajectory
demo sequence showcasing the full workspace.

This is ideal for screen-recording your portfolio demo video.

Usage:
    ros2 launch cobot_description demo_motion.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file]
    )

    # Demo motion publisher — runs after 3s to let RViz initialise
    demo_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='cobot_description',
                executable='demo_motion',
                name='demo_motion_node',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        demo_node,
    ])
