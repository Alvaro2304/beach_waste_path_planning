#!/bin/bash
set -e

# Fix ownership of named volumes (Docker creates them as root)
sudo chown -R $(id -u):$(id -g) /ros2_ws/build /ros2_ws/install /ros2_ws/log 2>/dev/null || true

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source workspace overlay if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Gazebo Harmonic resource path
export GZ_SIM_RESOURCE_PATH="/ros2_ws/src:${GZ_SIM_RESOURCE_PATH}"

exec "$@"
