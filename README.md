# sw_ros2_control

```
sudo apt install ros-foxy-ackermann-msgs
sudo apt install ros-foxy-tf2-tools -y
sudo apt install ros-foxy-ros2-control
sudo apt install ros-foxy-ros2-controllers
sudo apt install ros-foxy-gazebo-ros2-control
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

```
ros2 launch sw_ros2_control_gazebo racecar.launch.py 
ros2 run sw_ros2_control racecar_controller
ros2 run rqt_robot_steering rqt_robot_steering
=> /forward_position_controller/commands
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
- 0.5
- 0.5"

ros2 run sw_ros2_control racecar_controller
ros2 run sw_ros2_control ackermann_converter

ros2 launch sw_ros2_control_gazebo racecar_with_world.launch.py

# Walker World Spawn

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/kimsooyoung/ros2_ws/install/sw_ros2_control_gazebo/share/sw_ros2_control_gazebo

