from setuptools import find_packages, setup

package_name = 'vision_skips'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sanmaster5',
    maintainer_email='s.penunuri@hotmail.com',
    description="Package that manages the image processing of skip's camera in a server and sends the data to a client.",
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camara_compressed_client = vision_skips.camara_compressed_client:main',
            'travel_to_hazmat = vision_skips.travel_to_hazmat:main',
            'landmark_navigator = vision_skips.landmark_navigator:main'
        ],
    },
)
