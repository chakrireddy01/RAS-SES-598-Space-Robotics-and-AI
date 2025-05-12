# Assignment 3: Rocky Times Challenge - Search, Map, & Analyze
## Project Overview
This project was done as part of the Space Robotics and AI course to simulate a drone mission in rocky terrain using PX4, ROS2, and Ignition Gazebo. The goal was to make the drone take off, detect two cylindrical rocks, find the taller one, and land on it—all without using QGroundControl. Instead, the drone was controlled using ROS2 Python scripts and MAVROS. The simulation involved spawning the drone and cylinders, tracking visual markers using ArUco, and handling the drone’s behavior through custom scripts. Due to system limitations like low simulation speed (RTF), full landing was not always possible, but the setup and logic worked well in parts.

## Objective
The goal of this assignment was to simulate a PX4-powered drone that could autonomously:
- Take off from a flat surface
- Scan a rocky terrain for cylindrical structures
- Identify and compare two cylinders (of different heights)
- Estimate their dimensions using camera feed
- Land precisely on the taller cylinder

## Tools & Technologies Used:
1. **PX4 SITL (Software In The Loop)** – for drone flight control
2. **Ignition Gazebo** – to simulate realistic 3D environments
3. **ROS2 Jazzy** – middleware for modular communication
4. **MAVROS** – ROS2-MAVLink bridge
5. **OpenCV & ArUco** – for marker detection and localization
6. **Python (rclpy)** – for writing autonomous behavior nodes
7. **Launch files** – for streamlined multi-node deployment

## System Modules Overview:
- **simple_takeoff.py**: Handles OFFBOARD mode transition and drone arming. Publishes continuous position setpoints to initiate takeoff (without QGroundControl).
- **aruco_tracker.py**: Uses ArUco markers to detect cylinders and estimate their positions using ROS2 image and transform topics.
- **autonomous_landing.py**: Calculates which cylinder is taller based on pose data and commands the drone to align and descend safely onto it.
- **cylinder_landing.launch.py**: Orchestrates the full simulation stack:
  - Launches PX4 SITL with `make px4_sitl gz_x500_depth_mono`
  - Spawns both tall and short cylinder models
  - Starts necessary bridges between Gazebo and ROS2
  - Runs MAVROS for communication

## Execution Steps:
1. **Start PX4 and Gazebo manually**:
   ```bash
   make px4_sitl gz_x500_depth_mono
2. **Launch the system and simulation environment**:
   ```bash
   ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py
3. **Autonomous takeoff using ROS2 only (no QGroundControl)**:
   ```bash
   python3 simple_takeoff.py
4. **(When performance allowed) Execute marker tracking and autonomous landing**:
   ```bash
   python3 autonomous_landing.py
## Key Achievements:
 Successfully launched and connected the PX4 SITL environment

Cylinders were spawned into the simulated terrain

Autonomous take-off was achieved using ROS2 and MAVROS

No dependency on QGroundControl for mode switching or arming

The ArUco detection pipeline and landing logic were validated in past runs

## Challenges & Limitations:
Low Real-Time Factor (RTF): Due to system constraints, RTF stayed under ~5% (0.05), causing heavy simulation lag and dropped MAVLink messages.

Rendering Issues: Running in VirtualBox caused libEGL errors due to missing 3D acceleration, affecting the visualization of camera and sensor feeds.

Mode Flapping: Occasionally switched between OFFBOARD and LOITER due to delayed setpoints or dropped MAVROS topics.

## Conclusion:
Despite the limitations posed by low RTF and system performance, the project achieved key goals:

Verified autonomous drone takeoff via ROS2 scripts alone

All ArUco marker and landing scripts were implemented and functionally validated in earlier trials

System integration was successfully tested in RViz and confirmed using MAVROS topic echoes

The final system provides a complete autonomous flight pipeline and shows potential for full mission execution on higher-performance hardware.

## Photographs:
Simulation Environment Setup: ![image](https://github.com/user-attachments/assets/7ca2c9d7-0dfa-44b3-b10d-8e6c3ec6e02d)


Drone Simulation Code: ![image](https://github.com/user-attachments/assets/77d2efe2-fd8b-47fb-9952-f956b4b6313d)


Drone Spawn and Takeoff: ![image](https://github.com/user-attachments/assets/f34beffc-2c51-4d8b-b60d-f1c6bf5c5f92)
