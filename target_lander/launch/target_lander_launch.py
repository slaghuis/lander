import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('target_lander'),
        'config',
        'params.yaml'
        )
        
    node=Node(
        package = 'target_lander',
        name = 'lander',
        executable = 'target_lander',
        output="screen",
        emulate_tty=True,
        parameters = [config]
    )
    ld.add_action(node)
    return ld
