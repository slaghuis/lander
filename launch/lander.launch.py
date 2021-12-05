import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('lander'),
        'config',
        'params.yaml'
        )

    lander_node=Node(
        package = 'lander',
        name = 'lander',
        executable = 'lander_node',
        output="screen",
        emulate_tty=True,
        parameters = [config]
    )
    tracker_node=Node(
        package = 'lander',
        name = 'tracker',
        executable = 'tracker_node',
        output="screen",
        emulate_tty=True
    )
    ld.add_action(lander_node)
    ld.add_action(tracker_node)

    return ld
