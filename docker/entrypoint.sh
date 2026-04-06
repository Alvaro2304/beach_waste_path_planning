#!/bin/bash
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source workspace overlay if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Gazebo resource path
export GZ_SIM_RESOURCE_PATH="/ros2_ws/src:${GZ_SIM_RESOURCE_PATH}"

exec "$@"
