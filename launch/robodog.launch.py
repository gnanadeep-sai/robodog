"""
Brings up the full ROS 2 stack: the upstream `joy` driver node (which
replaces the legacy `PS4Joystick`/`PupperCommand` custom UDP socket
servers), `controller_node`, and `hardware_node`, all sharing the one
`config/pupper_params.yaml` parameter file.

Usage:
    ros2 launch robodog_ros2 robodog.launch.py
    ros2 launch robodog_ros2 robodog.launch.py params_file:=/path/to/your.yaml joy_dev:=/dev/input/js1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("robodog_ros2")
    default_params_file = os.path.join(pkg_share, "config", "pupper_params.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the ROS 2 parameters YAML for controller_node/hardware_node",
    )
    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev",
        default_value="/dev/input/js0",
        description="Joystick device consumed by the upstream joy_node",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"dev": LaunchConfiguration("joy_dev")}],
    )

    controller_node = Node(
        package="robodog_ros2",
        executable="controller_node",
        name="controller_node",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    hardware_node = Node(
        package="robodog_ros2",
        executable="hardware_node",
        name="hardware_node",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    return LaunchDescription(
        [
            params_file_arg,
            joy_dev_arg,
            joy_node,
            controller_node,
            hardware_node,
        ]
    )