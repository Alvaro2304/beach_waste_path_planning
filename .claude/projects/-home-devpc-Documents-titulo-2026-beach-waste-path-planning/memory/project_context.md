---
name: Project Context
description: Beach waste collection robot — 4-wheel diff-drive, IMU+GPS fusion, Nav2 navigation on sand terrain
type: project
---

Beach waste path planning robot project for thesis 2026.

**Why:** Autonomous beach cleaning — robot must navigate irregular sand terrain where wheel encoders slip.
**How to apply:** All localization must rely on IMU+GPS fusion (no encoder odometry). Nav2 config must account for outdoor GPS-based navigation (no pre-built maps, larger costmaps, GPS coordinate frames).

Key decisions:
- 4 actuated wheels, differential drive configuration
- Primary sensors: IMU + GPS (encoders unreliable on sand)
- Starting with EKF filter design, then Nav2 path planning
- Prior work in ../turtlebot4_path_planning serves as reference codebase
- Robot URDF/model being designed from scratch (still in design phase as of 2026-04-09)

Hardware:
- Dev PC: Ubuntu 22.04, NVIDIA GPU — Docker with ROS 2 Humble + Gazebo Fortress
- Dev laptop: also has NVIDIA GPU (used for coding while at work)
- Robot onboard: Raspberry Pi 4 (8GB) — runs ROS 2 Humble navigation stack
- Robot onboard: Jetson Nano (original 4GB, JetPack 4.x) — vision/AI only, NOT running ROS 2 nav
- GUI tools (Gazebo Fortress, RViz2, PlotJuggler) via X11 forwarding from Docker

Architecture:
- Jetson Nano: waste detection / computer vision
- RPi4: ROS 2 Humble, EKF, Nav2, motor control
- Communication between Jetson and RPi4 TBD

IMPORTANT: Changed from RPi5 + Jazzy to RPi4 + Humble on 2026-04-09.
Gazebo version changed from Harmonic to Fortress accordingly.
Ignition-era package naming applies (ign-ros2-control, IGN_GAZEBO_RESOURCE_PATH, etc.)
