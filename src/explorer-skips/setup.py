from setuptools import find_packages, setup

package_name = 'turtlebot3_left_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/exploration.launch.py']),
        ('share/' + package_name + '/config', ['config/navigation.yaml', 'config/slam.yaml'])        


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='gerardosanchezz14@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'left_wall_follower = turtlebot3_left_follower.left_wall_follower:main',
            'maze_escape_robot = turtlebot3_left_follower.maze_escape_robot:main',
            'pledge_algorithm = turtlebot3_left_follower.pledge_algorithm:main'
            'nav_goal_sender = turtlebot3_left_follower.nav_goal_sender:main'
        ],
    },
)
