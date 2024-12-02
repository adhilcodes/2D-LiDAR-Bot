# 2D-LiDAR SLAM Robot 

## Overview
A ROS1 Noetic-based differential drive robot with LiDAR SLAM capabilities for autonomous navigation.

## Software Requirements
- ROS1 Noetic
- Gazebo
- SLAM Toolbox
- Xacro
- RViz

## Hardware Setup
<will update here later!>

## Package Structure
```
lidarbot/
├── launch/
│   ├── launch_sim.launch
│   └── rsp.launch
├── urdf/
│   ├── robot.urdf.xacro
│   ├── robot_core.xacro
│   ├── inertial_macros.xacro
│   ├── gazebo_control.xacro
│   └── lidar.xacro
├── scripts/
│   └── lidar_check.py
├── config/
│   └── mapper_params_online_async.yaml
└── worlds/
    └── empty.world
```

## Installation
```bash
cd ~/catkin_ws/src
git clone <repository_url>
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
   
## Running Simulation
```bash
roslaunch lidarbot launch_sim.launch
roslaunch slam_toolbox online_async.launch params_file:=./src/lidarbot/config/mapper_params_online_async.yaml use_sim_time:=true
   ```

## Configuration
- Modify `urdf/robot.urdf.xacro` for robot description
- Adjust `config/mapper_params_online_async.yaml` for SLAM parameters

## Troubleshooting
- Ensure correct TF frames
- Check LiDAR and odometry configurations

## Future Improvements
- Add navigation stack
- Implement autonomous navigation
