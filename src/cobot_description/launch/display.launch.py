import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = FindPackageShare(package='cobot_description').find('cobot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'cobot.urdf.xacro')
    rviz_file = os.path.join(pkg_share, 'rviz', 'cobot_view.rviz')

    declare_gui_arg = DeclareLaunchArgument(
        name='gui', default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
    declare_rviz_arg = DeclareLaunchArgument(
        name='rviz', default_value='true',
        description='Flag to launch RViz2'
    )
    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    }
    robot_state_publisher_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen', parameters=[robot_description]
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui', output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        name='joint_state_publisher', output='screen',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_file], condition=IfCondition(LaunchConfiguration('rviz'))
    )
    return LaunchDescription([
        declare_gui_arg, declare_rviz_arg,
        robot_state_publisher_node, joint_state_publisher_gui_node,
        joint_state_publisher_node, rviz_node,
    ])
