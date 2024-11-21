from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    explorer_skips_dir = get_package_share_directory('explorer_skips')

    # Define launch arguments
    nav2_params = os.path.join(explorer_skips_dir, 'config', 'navigation.yaml')
    slam_params = os.path.join(explorer_skips_dir, 'config', 'slam_localization.yaml')

    return LaunchDescription([
        # Nodo de rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(explorer_skips_dir, 'rviz', 'explore.rviz')]        
        ),
        # Nodo de map_server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': '/home/ubuntu/maps/map.yaml'}]
        ),
        # Cambiar el estado de map_server a 'configuring'
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/map_server/change_state', 
                'lifecycle_msgs/srv/ChangeState', 
                "{transition: {id: 1, label: ''}}"
            ],
            shell=True
        ),
        # Cambiar el estado de map_server a 'active'
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/map_server/change_state', 
                'lifecycle_msgs/srv/ChangeState', 
                "{transition: {id: 3, label: ''}}"
            ],
            shell=True
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
        # Nodo para nav goals
        Node(
           package='explorer_skips',
           executable='nav_goal_sender',
           name='nav_goal_sender',
           output='screen'
        ),
        # Nodo de exploración para encontrar simbolos 
        Node(
            package='explorer_skips',
            executable='random_exploration',
            name='random_exploration'
        )
    ])
