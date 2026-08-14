import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("sleepy_lio")

    return LaunchDescription(
        [
            # LM 可单独调试；实际参数文件建议由调用方显式传入。
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(package_share, "config", "mid360.yaml"),
                description="LM parameter file",
            ),
            Node(
                package="sleepy_lio",
                executable="sleepy_lio_node",
                name="sleepy_lio_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
