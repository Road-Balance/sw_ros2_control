# Author: Addison Sears-Collins
# Date: August 31, 2021
# Description: Launch a basic mobile robot
# https://automaticaddison.com

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Set the path to different files and folders.
    pkg_share = os.path.join(get_package_share_directory('sw_sensor_fusion'))

    # Specify the actions

    # Start Gazebo server
    start_example_env = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_share, 'launch', 'racecar_with_world.launch.py'
                )
            ),
    )

    # Start robot localization using an Extended Kalman filter
    robot_localization_file_path = os.path.join(pkg_share, 'config', 'ekf.yaml') 
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            robot_localization_file_path, 
            {'use_sim_time': True}
        ]
    )

    rviz_config_file = os.path.join(pkg_share, 'rviz', 'robot_localization.rviz')

    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        start_example_env,
        robot_localization,
        rviz,
    ])