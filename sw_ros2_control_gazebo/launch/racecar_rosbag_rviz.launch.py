import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('sw_ros2_control_gazebo'))
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'gazebo.rviz')

    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rviz,
    ])