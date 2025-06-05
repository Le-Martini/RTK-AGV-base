from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agribot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web'), glob('web/*'))
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'pyserial', 'pynmea2', 
                     'Adafruit_BNO055', 'geometry_msgs', 'std_msgs', 'numpy', 'scipy',
                     'pyproj', 'nav_msgs', 'geographic_msgs', 'robot_localization'],
    zip_safe=True,
    maintainer='LeMartini',
    maintainer_email='lmn19052001@gmail.com',
    description='ROS2 nodes for AgriBot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_node = agribot.esp32_node:main',
            'gps_node = agribot.gps_node:main',
            'imu_node = agribot.imu_node:main',
            'localization = agribot.localization:main',
            'gps_to_odom = agribot.gps_to_odom:main',
            'main_control = agribot.main_control:main'
        ],
    },
)
