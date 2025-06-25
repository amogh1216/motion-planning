import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Set the package and URDF file name
    pkg_name = 'robot'
    config_file = 'robot-config.rviz'
    rviz_config_path = PathJoinSubstitution([FindPackageShare(pkg_name), 'config', config_file])

    urdf_file = 'four_wheel_robot.xacro'

    # Build the full path to the URDF file
    urdf_path = PathJoinSubstitution([FindPackageShare('robot'), 'urdf', urdf_file])

    launch_gazebo = IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
            launch_arguments={
                'urdf_package': 'robot',
                'urdf_package_path': urdf_path,
                'rviz_config': rviz_config_path
            }.items()
        )

    return LaunchDescription([
        launch_gazebo
    ])
