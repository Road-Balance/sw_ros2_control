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

    # Prepare Robot State Publisher Params
    urdf_file = os.path.join(pkg_path, 'urdf', 'racecar', 'racecar.urdf')
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_config_file = os.path.join(pkg_path, 'rviz', 'description.rviz')

    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_state_publisher,
        #         on_exit=[joint_state_publisher_gui],
        #     )
        # ),
        joint_state_publisher,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])