from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    explorer_skips_dir = get_package_share_directory('explorer_skips')

    # Define launch arguments
    nav2_params = os.path.join(explorer_skips_dir, 'config', 'navigation.yaml')
    slam_params = os.path.join(explorer_skips_dir, 'config', 'slam.yaml')

    declared_arguments = []


    return LaunchDescription(declared_arguments + [
        Node(
            package='tf2_ros',
            namespace = 'scan_to_map',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "map", "scan"]
        ),
        # Nodo de rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(explorer_skips_dir, 'rviz', 'explore.rviz')]        
            ),
        # Nodo de SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params],
            output='log'
        ),
        # Lanzar Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': nav2_params, 'output': 'log'}.items(),
        ),
        # Nodo para marcadores
        Node(
           package='explorer_skips',
           executable='landmark_marker',
           name='landmark_marker',
           output='screen'
        ),
        # Nodo para nav goals
        Node(
           package='explorer_skips',
           executable='nav_goal_sender',
           name='nav_goal_sender',
           output='screen'
        ),
        # Tu nodo de exploración
        # Node(
        #    package='explorer_skips',
        #    executable='wavefront_planner',
        #    name='wavefront_planner',
        #    output='screen'
        # )
    ])