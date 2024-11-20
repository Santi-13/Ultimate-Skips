from setuptools import find_packages, setup

package_name = 'explorer_skips'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/exploration.launch.py', 'launch/left_wall_exploration.launch.py']),
        ('share/' + package_name + '/config', ['config/navigation.yaml', 'config/slam_mapping.yaml', 'config/slam_localization.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/explore.rviz']),  

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sanmaster5',
    maintainer_email='s.penunuri@hotmail.com',
    description="Package that manages skip's mapping, navigation and exploration algorithms.",
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'left_wall_follower = explorer_skips.left_wall_follower:main',
            'nav_goal_sender = explorer_skips.nav_goal_sender:main',
            'frontier_analyzer = explorer_skips.frontier_analyzer:main',
            'wavefront_planner = explorer_skips.wavefront_planner:main',
            'landmark_marker = explorer_skips.landmark_marker:main', 
            'random_exploration = explorer_skips.random_exploration:main',
            'explore = explorer_skips.explore:main'
        ],
    },
)
