# Only Gazebo & Spawn

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

import xacro

def generate_launch_description():
    # robot state publisher


    # # To check
    # # robot_description = {"robot_description": robot_description_content}

    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("my_rotate_bot"),
    #         "controllers",
    #         "testbot_controller.yaml",
    #     ]
    # )



    # node_joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher'
    # )

    # gazebo
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    pkg_gazebo_ros, 'launch', 'gazebo.launch.py'))
            )

    # Robot State Publisher
    pkg_path = os.path.join(get_package_share_directory('sw_ros2_control_gazebo'))
    urdf_file = os.path.join(pkg_path, 'urdf', 'model.urdf')

    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # spawn robot
    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'rotate_box_bot'],
    #                     output='screen')

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        # spawn_entity,
        # node_joint_state_publisher,
    ])
