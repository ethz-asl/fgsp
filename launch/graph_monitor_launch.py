from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    monitor_node = Node(
        package="fgsp",
        executable="graph_monitor",
        output={'full': 'screen'},
        emulate_tty=True,
    )
    ld.add_action(monitor_node)

    return ld