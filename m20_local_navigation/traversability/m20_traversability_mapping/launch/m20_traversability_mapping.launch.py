from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("m20_traversability_mapping")) / "config" / "m20_traversability_mapping.yaml"
    return LaunchDescription([
        Node(
            package="m20_traversability_mapping",
            executable="traversability_mapping_node",
            name="m20_traversability_mapping",
            output="screen",
            parameters=[str(config)],
        )
    ])
