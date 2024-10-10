from setuptools import find_packages, setup

package_name = 'ultimate_skips'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fake.launch.py','launch/robot.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/burger.gazebo.xacro', 'urdf/burger.urdf.xacro', 'urdf/materials.xacro', 'urdf/turtlebot3_burger.urdf.xacro', 'urdf/turtlebot3_system.ros2_control.xacro']),
        ('share/' + package_name + '/meshes', ['meshes/burger_base.stl', 'meshes/lds.stl', 'meshes/left_tire.stl', 'meshes/right_tire.stl']),
        ('share/' + package_name + '/worlds', []),
        ('share/' + package_name + '/rviz', ['rviz/turtlebot3.rviz']),
        ('share/' + package_name + '/config', ['config/gazebo_controller_manager.yaml', 'config/hardware_controller_manager.yaml'])        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanmaster',
    maintainer_email='s.penunuri@hotmail.com',
    description='A simple SLAM and map navigation implementation on a turtlebot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello = ultimate_skips.hello:main'
        ],
    },
)
