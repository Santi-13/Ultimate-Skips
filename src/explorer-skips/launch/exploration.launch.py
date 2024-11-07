from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    #nav2_params = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
    #slam_params = os.path.join(slam_toolbox_dir, 'config', 'mapper_params_online_async.yaml')

    bringup_dir = get_package_share_directory('turtlebot3_left_follower')

    nav2_params = os.path.join(bringup_dir, 'config', 'navigation.yaml')
    slam_params = os.path.join(bringup_dir, 'config', 'slam.yaml')


    return LaunchDescription([
        # Node de rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('tutorial_pkg'), 'rviz', 'explore.rviz')]        
            ),
        # Nodo de SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        ),
        # Lanzar Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': nav2_params}.items(),
        )
        # Tu nodo de exploración
        #Node(
        #    package='autonomous_explore_cpp',
        #    executable='autonomous_explore',
        #    name='autonomous_explore',
        #    output='screen'
        #),
    ])