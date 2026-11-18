import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robodog_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "transforms3d", "pigpio"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO@example.com",
    description=(
        "ROS 2 migration of the legacy StanfordQuadruped/pupper control "
        "stack: pigpiod-based hardware layer, unmodified gait/IK core, "
        "and rclpy node wrappers."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # ROS 2 nodes.
            "controller_node = robodog_ros2.nodes.controller_node:main",
            "hardware_node = robodog_ros2.nodes.hardware_node:main",
            # Headless bring-up CLIs -- no ROS dependency, run directly
            # on the Pi outside of colcon/ros2 run.
            "calibration_cli = robodog_ros2.hardware.calibration_cli:main",
            "i2c_diagnostics = robodog_ros2.hardware.i2c_diagnostics:main",
        ],
    },
)