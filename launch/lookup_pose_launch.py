import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('fgsp'),
        'config',
        'lookup_aligned_pose.yaml'
    )

    lookup_node = Node(
        package="fgsp",
        executable="lookup_aligned_pose",
        output={'both': 'screen'},
        emulate_tty=True,
        parameters=[config]
    )
    ld.add_action(lookup_node)

    return ld
