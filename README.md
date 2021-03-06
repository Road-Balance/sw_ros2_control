# sw_ros2_control

```
source /usr/share/gazebo/setup.sh
```

### Install Dependencies

```
sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup -y
sudo apt install ros-foxy-turtlebot3* -y
sudo apt install python3-rosdep2 -y
rosdep update

sudo apt install ros-foxy-rviz-common \
                ros-foxy-joint-state-publisher-gui \
                ros-foxy-gazebo-ros2-control \
                ros-foxy-rqt-robot-steering \
                ros-foxy-robot-localization \
                ros-foxy-cartographer-ros \
                ros-foxy-ros2-controllers \
                ros-foxy-gazebo-ros-pkgs \
                ros-foxy-ackermann-msgs \
                ros-foxy-ros2-control \
                ros-foxy-gazebo-ros \
                ros-foxy-tf2-tools \
                ros-foxy-xacro \
                ros-foxy-slam-toolbox -y

rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
```


### Build

```
delete racecar.urdf 
delete clean_racecar.urdf 
comment 24 - 38 line in CMakeLists.txt in sw_ros2_control_gazebo

cbp sw_ros2_control_gazebo
rosfoxy
cbp sw_ros2_control 
rosfoxy
cbp rviz_plugin_tutorials
rosfoxy
cbp rqt_rc_steering
rosfoxy


colcon build --symlink-install --packages-select sw_ros2_control
colcon build --symlink-install --packages-select sw_ros2_control_gazebo
```

## View the Coordinate Frames

```
ros2 run tf2_tools view_frames.py
evince frames.pdf
``` 


```
$ ros2 control list_controller_types
controller_manager/test_controller
             controller_interface::ControllerInterface
controller_manager/test_controller_failed_init
             controller_interface::ControllerInterface
controller_manager/test_controller_with_interfaces                     controller_interface::ControllerInterface
diff_drive_controller/DiffDriveController
             controller_interface::ControllerInterface
effort_controllers/JointGroupEffortController
             controller_interface::ControllerInterface
force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster           controller_interface::ControllerInterface
forward_command_controller/ForwardCommandController                    controller_interface::ControllerInterface
imu_sensor_broadcaster/IMUSensorBroadcaster
             controller_interface::ControllerInterface
joint_state_broadcaster/JointStateBroadcaster
             controller_interface::ControllerInterface
joint_state_controller/JointStateController
             controller_interface::ControllerInterface
joint_trajectory_controller/JointTrajectoryController                  controller_interface::ControllerInterface
position_controllers/JointGroupPositionController
             controller_interface::ControllerInterface
velocity_controllers/JointGroupVelocityController
             controller_interface::ControllerInterface
```

# Joint Trajectory Controller

```
ros2 launch sw_ros2_control_gazebo boxbot_example_position_traj.launch.py 
```

```
$ ros2 control list_hardware_interfaces
command interfaces
        link_1_JOINT_0/position [claimed]
state interfaces
         link_1_JOINT_0/effort
         link_1_JOINT_0/position
         link_1_JOINT_0/velocity
```

```
$ ros2 control list_controllers
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
```

```
$ ros2 action list
/joint_trajectory_controller/follow_joint_trajectory
```

# Forward Command Controller 

```
ros2 launch sw_ros2_control_gazebo boxbot_example_position_forward.launch.py 
```

```
$ ros2 control list_hardware_interfaces
command interfaces
        link_1_JOINT_0/position [claimed]
state interfaces
         link_1_JOINT_0/effort
         link_1_JOINT_0/position
         link_1_JOINT_0/velocity
```

```
$ ros2 control list_controllers
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
forward_position_controller[forward_command_controller/ForwardCommandController] active
```

```
$ ros2 topic list
/clock
/dynamic_joint_states
/forward_position_controller/commands
/joint_states
/parameter_events
/performance_metrics
/robot_description
/rosout
/tf
/tf_static
```

# Velocity Controllers

```
ros2 launch sw_ros2_control_gazebo boxbot_example_velocity.launch.py
```

```
$ ros2 control list_controllers
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
velocity_controller [velocity_controllers/JointGroupVelocityController] active
```

```
$ ros2 topic list
/clock
/dynamic_joint_states
/joint_states
/parameter_events
/performance_metrics
/robot_description
/rosout
/tf
/tf_static
/velocity_controller/commands
```

```
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data:
- 2.0"

ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data:
- -1.0"
```

## Control with cmd_vel

```
ros2 run sw_ros2_control boxbot_controller
```

# Racecar Gazebo

* description
```

ros2 launch sw_ros2_control_gazebo racecar_description.launch.py
```


```
ros2 launch sw_ros2_control_gazebo racecar.launch.py 
ros2 launch sw_ros2_control racecar_control.launch.py
ros2 run rqt_robot_steering rqt_robot_steering
```

ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.5
- 0.5"

ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
- -0.5
- -0.5"


ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.5
- 0.5
- 0.5- 0.5"


# Walker World Spawn

```
cd <ros2-ws>/src/sw_ros2_control/sw_ros2_control_gazebo/models
cp -r walker_racecourse ~/.gazebo/models
source ~/.bashrc
```

```
ros2 launch sw_ros2_control_gazebo racecar_with_world.launch.py
```

# Sensor Fusion

gazebo + racecar + robot_localization + rviz

```
ros2 launch sw_sensor_fusion racecar_with_world.launch.py

# only robot_localization + rviz
ros2 launch sw_sensor_fusion robot_localization.launch.py
```

# SLAM - Cartographer 

```
ros2 launch sw_cartographer_slam racecar_with_world.launch.py
ros2 launch sw_cartographer_slam cartographer.launch.py 
# only slam + rviz

# move racecar with rqt gui 

ros2 launch sw_cartographer_slam racecar_with_world.launch.py
ros2 bag play racecar_cmd_vel
ros2 launch sw_cartographer_slam robot_localization.launch.py
ros2 launch sw_cartographer_slam cartographer.launch.py 

# Save Map
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: my_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"

# Map lanunch
ros2 launch cartographer_slam nav2_map_server_launch.py

```

# SLAM - slam_toolbox

```
ros2 launch sw_slam_toolbox racecar_with_world.launch.py
ros2 launch sw_slam_toolbox online_async_launch.py
```
 