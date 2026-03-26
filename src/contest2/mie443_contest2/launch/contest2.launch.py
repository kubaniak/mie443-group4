import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mie443_contest2'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='mie443_contest2',
            executable='contest2',
            name='contest2',
            output='screen',
            parameters=[config]
        )
    ])
