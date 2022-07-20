import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('reproject_viz'),
        'config',
        'config.yaml'
    )

    ply_pub = Node(
        package="reproject_viz",
        executable="reproject_viz",
        output={'full': 'screen'},
        emulate_tty=True,
        parameters=[config]
    )
    ld.add_action(ply_pub)

    return ld
