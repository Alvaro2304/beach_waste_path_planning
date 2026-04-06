---
name: Reference TurtleBot4 Package
description: Prior ROS2 package at ../turtlebot4_path_planning with custom EKF (IMU+encoder), Nav2 waypoint nav, Gazebo sim
type: reference
---

Located at: /home/devpc/Documents/titulo_2026/turtlebot4_path_planning

Contains:
- Custom 5-state EKF (x,y,theta,v,omega) fusing IMU + encoder odometry (C++)
- Nav2 config with NavfnPlanner + DWB local planner
- Waypoint navigator and initial pose publisher nodes
- TurtleBot4 Gazebo Ignition simulation with custom URDF (includes IMU plugin)
- SLAM toolbox integration
- Diff-drive controller config (wheel_sep=0.233m, wheel_rad=0.03575m)
