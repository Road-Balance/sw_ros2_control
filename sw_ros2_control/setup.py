from glob import glob

import os

from setuptools import setup

package_name = 'sw_ros2_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimsooyoung',
    maintainer_email='tge1375@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boxbot_controller = sw_ros2_control.boxbot_controller:main',
            'racecar_controller = sw_ros2_control.racecar_controller:main',
            'ackermann_converter = sw_ros2_control.cmd_vel_to_ackermann_drive:main',
        ],
    },
)
