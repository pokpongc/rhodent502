# Rhodent Mobile Robot Repository
## Dependencies
- gazebo_ros
- ros_control
- cartographer_ros
- anytree
- pocketsphinx
- pyaudio
- cob_gazebo_objects
- move_base
- pygame

## Useful Commands
Launching in SLAM mode without using voice command:
```bash
roslaunch rhodent_navigation auto_navigation.launch
```

Launching in localization mode (use a recorded map) without using voice command:
```bash
roslaunch rhodent_navigation auto_navigation.launch localization_mode:=pure_localization
```

Launching in voice command mode:
```bash
roslaunch rhodent_navigation voice_navigation.launch
```

## ROS Topics
### High-level Control
Action | ROS Topic | Message Type
------------ | ------------- | -------------
Setting desired room destination | /rhodent/room_cmd | std_msgs/String
Via point visualization | /rhodent/via_point | visualization_msgs/MarkerArray
Setting desired pose | /move_base/goal | move_base_msgs/MoveBaseActionGoal

### Actuation
Action | ROS Topic | Message Type
------------ | ------------- | -------------
Base velocity command | /rhodent/cmd_vel | geometry_msgs/Twist
Wheel velocity command | /rhodent/{side}_wheel_velocity_controller/command | std_msgs/Float64

### Sensing
Sensor | ROS Topic | Message Type
------------ | ------------- | -------------
Inertial Measurement Unit | /imu | sensor_msgs/Imu
LIDAR output | /scan | sensor_msgs/LaserScan
Reading joints' position and velocity | /rhodent/joint_states | sensor_msgs/JointState

## ROS Graph
![Alt text](/media/rosgraph.png?raw=true)
