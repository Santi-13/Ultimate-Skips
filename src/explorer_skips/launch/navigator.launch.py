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
    slam_params = os.path.join(explorer_skips_dir, 'config', 'slam_localization.yaml')
    map_file = os.path.join(explorer_skips_dir, 'maps', 'map.yaml')
    rviz_config = os.path.join(explorer_skips_dir, 'rviz', 'explore.rviz')

    return LaunchDescription([
        # Publicador de transformaciones estáticas
        Node(
            package='tf2_ros',
            namespace='scan_to_map',
            executable='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "map", "scan"]
        ),
        # Nodo de RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        ),
        # Nodo de filtro de láser
        Node(
            package='explorer_skips',
            executable='laser_scan_filter_node',
            name='laser_scan_filter_node',
            output='screen'
        ),
        # Nodo de SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params],
            output='log'
        ),
        # Lanzar Navigation2 con inicialización del mapa
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': nav2_params,
                'map': map_file,  # Agrega el mapa
                'use_sim_time': 'false'
            }.items()
        ),
        # Nodo para marcadores
        Node(
            package='explorer_skips',
            executable='landmark_marker',
            name='landmark_marker',
            output='screen'
        )
    ])
