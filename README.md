# Rhodent Mobile Robot Repository
FRA502 "Mobile Robotics" 2020
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

Saving map:
```bash
rosservice call /write_state ${path_to_src}/rhodent502/rhodent_navigation/map/apartment_map.pbstream
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

## Resources
- [Transformation Tree](https://github.com/pokpongc/rhodent502/blob/master/media/frames.pdf)
- [Recorded Map (*.pbstream)](https://github.com/pokpongc/rhodent502/blob/master/rhodent_navigation/map/apartment_map.pbstream)
- [Voice Command Weight File](https://github.com/pokpongc/rhodent502/tree/master/rhodent_interface/config)
- [Navigation and Mapping Config Files](https://github.com/pokpongc/rhodent502/tree/master/rhodent_navigation/config)
- [Low-level Controller Config File](https://github.com/pokpongc/rhodent502/blob/master/rhodent_gazebo/config/controller.yaml)
