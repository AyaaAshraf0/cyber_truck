# Cyber Truck Simulation 

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo Ignition](https://img.shields.io/badge/Gazebo-Ignition-orange?logo=gnometerminal&logoColor=white)](https://gazebosim.org/)
[![Nav2](https://img.shields.io/badge/Navigation2-Nav2-blue?logo=ros&logoColor=white)](https://navigation.ros.org/)

---

## 📖 Overview
This repository contains a ROS 2 Humble-based simulation of a **Cyber Truck-inspired mobile robot**.  
The robot is modeled with a custom URDF exported from SolidWorks, runs in **Gazebo Ignition**, and supports **autonomous navigation with Nav2**.

The repo includes:
- **`cyber_robt`** → robot description, URDF/Xacro, meshes, worlds, and launch files.  
- **`navigation`** → localization & navigation (Nav2, AMCL, map server).  
- **`fusion_odom_imu`** → used for imu and odom fusion using extended kalman filter.  

---

## ⚙️ Installation
1. Clone the repository into your ROS 2 workspace:
   ```bash
   cd ~/your_ws/src
   git clone https://github.com/AyaaAshraf0/cyber_truck.git
   cd ~/your_ws
   colcon build
   source install/setup.bash
2. install dependencies:
   ```bash
   sudo apt update
   rosdep install --from-paths src --ignore-src -r -y
   
## 🚀 Usage

### 1️⃣ Launch Gazebo with Cyber Truck
Start the Ignition Gazebo simulator with the Cyber Truck robot:
  ```bash
  ros2 launch cyber_robt ign.launch.py
  ```
Expected output:
Ignition Gazebo opens with the custom wider_classroom.sdf world.
The Cyber Truck robot spawns in the environment.

### 2️⃣ Visualize in RViz2
Make sure you are inside your workspace, run:
```bash
cd /path/to/ws
```
```bash
rviz2 -f src/cyber_robt/config/cyber_truck.rviz 
```
Expected output:
RViz2 opens.
Robot model, TF tree, and laser scan are displayed.
Map and navigation goals can be visualized.

### 3️⃣ Run Navigation (Nav2 Stack)
Bring up the Navigation2 stack with:
```bash
ros2 launch navigation navigation_bringup.launch.py
```
**Expected output:**  

- **Map Server** loads `wider_classroom_map.yaml`.  
  - Ensure your **global frame** is set to `map`.  
  - Use the **2D Pose Estimate** tool in RViz to initialize the robot pose.  
  - Alternatively, you can publish directly to `/initialpose` topic.  

- **AMCL Node** starts localizing the robot.  

- **Nav2 Lifecycle Manager** automatically activates `map_server` and `amcl`.  

- **RViz2 (if running)** displays:  
  - The map  
  - The robot’s current position  
  - Laser scan data  

- 🎯 You can send navigation goals using the **2D Goal Pose** tool in RViz.
  
### 4️⃣ Teleoperation (Manual Control)
Control the robot manually using the keyboard teleop node:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Expected output:
Keyboard inputs move the Cyber Truck inside the Gazebo world.
/cmd_vel topic is published and consumed by the robot.
 
