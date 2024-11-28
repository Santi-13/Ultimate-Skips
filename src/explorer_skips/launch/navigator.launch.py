from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    explorer_skips_dir = get_package_share_directory('explorer_skips')

    # Define paths
    nav2_params = os.path.join(explorer_skips_dir, 'config', 'navigation.yaml')
    map_file = os.path.join(explorer_skips_dir, 'maps', 'map.yaml')
    rviz_config = os.path.join(explorer_skips_dir, 'rviz', 'explore.rviz')

    return LaunchDescription([
        # Static transform from 'odom' to 'base_link'
        Node(
            package='tf2_ros',
            namespace='scan_to_map',
            executable='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "map", "scan"]
        ),
        # Nodo para marcadores
        Node(
           package='explorer_skips',
           executable='landmark_marker',
           name='landmark_marker',
           output='screen'
        ),
        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        
        # Navigation2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'params_file': nav2_params,
                'map': map_file,
                'use_sim_time': 'false'
            }.items()
        ),

        # Landmark Marker Node
        Node(
           package='explorer_skips',
           executable='landmark_marker',
           name='landmark_marker',
           output='screen'
        )
    ])
