from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    racecar_control = Node(
                        package='sw_ros2_control',
                        executable='racecar_controller',
                        name='racecar_controller',
                        output='screen'
                        )

    cmd_vel_to_ackermann_drive = Node(
                        package='sw_ros2_control',
                        executable='ackermann_converter',
                        name='ackermann_converter',
                        output='screen'
                        )

    return LaunchDescription([
        racecar_control,
        cmd_vel_to_ackermann_drive,
    ])