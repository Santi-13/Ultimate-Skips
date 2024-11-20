from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': '/home/ubuntu/maps/map.yaml'}]
        ),
        Node(
            package='exploration_skips',
            executable='random_exploration.py',
            name='random_exploration'
        )
    ])
