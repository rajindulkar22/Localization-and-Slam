#  ROS 2 SLAM & Navigation from Scratch

![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue) ![Python](https://img.shields.io/badge/Language-Python_3-yellow) ![Gazebo](https://img.shields.io/badge/Sim-Gazebo-orange) ![Nav2](https://img.shields.io/badge/Nav2-Stack-green)

##  Overview
This repository documents a comprehensive engineering journey into **ROS 2 Robotics programming**. 

Rather than simply using pre-made packages, this project focused on **first-principles engineering**. I built core robotics algorithms (Occupancy Grid Mapping, Particle Filters) from scratch in Python before integrating standard industry tools (SLAM Toolbox, Nav2) for the final autonomous system.

**Final Result:** A TurtleBot 4 simulation capable of fully autonomous frontier-based exploration and mapping in unknown environments.

---

##  Architecture & Modules

###  Phase 1: Perception & Kinematics
* **Raw Lidar Processing:** Developed nodes to read raw `/scan` data, converting Polar $(r, \theta)$ measurements to Cartesian $(x,y)$ coordinates for obstacle detection.
* **TF2 Transforms:** Implemented listeners to mathematically transform sensor data from the `laser_link` to the `base_link` frame, compensating for sensor mounting offsets.
* **Noise Filtering:** Applied Moving Average filters to clean noisy sensor readings before processing.

###  Phase 2: Mapping from Scratch
* **Custom Mapper Node:** Built a manual publisher for `nav_msgs/OccupancyGrid` messages.
* **Raycasting:** Implemented **Bresenham’s Line Algorithm** to simulate laser beams clearing free space between the robot and the detected wall.
* **Probabilistic Updates:** Used **Log-Odds (Bayes Filter)** math to update cell occupancy probabilities dynamically based on new sensor data.
* **Map Artifacts:** Handled grid resolution, origin offsets, and coordinate conversion (World Meters $\leftrightarrow$ Grid Pixels).

###  Phase 3: Localization / The Particle Filter
* **Monte Carlo Localization (MCL):** Coded a custom localization engine from the ground up.
* **Motion Model:** Propagated particle states based on noisy odometry data (handling drift).
* **Sensor Model:** Implemented Likelihood Fields to weight particles based on how well their simulated scans matched the real world.
* **Resampling:** Implemented "Survival of the Fittest" (Low variance resampling) to converge on the robot's true pose.

###  Phase 4: Autonomy & Exploration
* **Graph SLAM:** Configured `slam_toolbox` for loop closure and consistent large-scale mapping.
* **Nav2 Integration:** Tuned Global and Local Costmap inflation layers to allow safe navigation in tight mazes.
* **Frontier Exploration:** Developed a custom Python node (`explorer.py`) that:
    1.  Analyzes the map to find "Frontiers" (safe cells adjacent to unknown space).
    2.  Converts grid indices to real-world coordinates.
    3.  Commands the robot via Action Client to explore these boundaries autonomously.

---
###  Prerequisites
* ROS 2 Humble
* Gazebo Ignition
* TurtleBot 4 Simulation Packages

