from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    bringup_dir = get_package_share_directory('explorer_skips')

    slam_params = os.path.join(bringup_dir, 'config', 'slam.yaml')

    return LaunchDescription([
        # Node de rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(bringup_dir, 'rviz', 'explore.rviz')]        
            ),
        # Nodo de SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        ),
        # Left wall follower node
        Node(
           package='explorer_skips',
           executable='left_wall_follower',
           name='left_wall_follower',
           output='screen'
        ),
    ])