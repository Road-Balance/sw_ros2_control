import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

import xacro

def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('sw_ros2_control_gazebo'))
    world_file = os.path.join(pkg_path, 'worlds', 'racecar_walker.world')

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    pkg_gazebo_ros, 'launch', 'gazebo.launch.py'))
            )

    # Robot State Publisher
    pkg_path = os.path.join(get_package_share_directory('sw_ros2_control_gazebo'))
    urdf_file = os.path.join(pkg_path, 'urdf', 'racecar', 'racecar.urdf')

    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Spawn Robot
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'racecar'],
                        output='screen')

    return LaunchDescription([
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        # robot_state_publisher,
        # joint_state_publisher,
        # spawn_entity,
    ])