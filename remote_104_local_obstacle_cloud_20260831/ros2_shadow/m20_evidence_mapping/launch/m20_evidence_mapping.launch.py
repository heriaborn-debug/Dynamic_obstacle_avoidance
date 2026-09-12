from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("m20_evidence_mapping")) / "config" / "m20_evidence_mapping.yaml"
    return LaunchDescription(
        [
            Node(
                package="m20_evidence_mapping",
                executable="evidence_mapping_node",
                name="m20_evidence_mapping",
                output="screen",
                parameters=[str(config)],
            )
        ]
    )
