from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    explorer_skips_dir = get_package_share_directory('explorer_skips')

    # Define launch arguments
    world_arg = DeclareLaunchArgument('world', default_value=os.path.join(explorer_skips_dir, 'worlds', 'map_world.world'))

    nav2_params = os.path.join(explorer_skips_dir, 'config', 'navigation.yaml')
    slam_params = os.path.join(explorer_skips_dir, 'config', 'slam.yaml')



    return LaunchDescription([
        Node(
            package='tf2_ros',
            namespace = 'scan_to_map',
            executable='static_transform_publisher',
            arguments= ["0", "0", "0", "0", "0", "0", "map", "scan"]
        ),
        # Node de rviz2
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
            output='screen',
            parameters=[slam_params]
        ),
        # Lanzar Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': nav2_params}.items(),
        ),
        # Tu nodo de exploración
        # Node(
        #    package='explorer_skips',
        #    executable='nav_goal_sender',
        #    name='nav_goal_sender',
        #    output='screen'
        # ),
        # Gazebo server node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'))
        )
    ])