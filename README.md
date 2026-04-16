# Beach Waste Path Planning

Autonomous navigation stack for a beach waste collection robot. Built on ROS 2 Jazzy with Gazebo Harmonic simulation.

## Robot Platform

- **Drive:** 4 actuated wheels, differential drive configuration
- **Environment:** Beach / sand terrain (irregular surface)
- **Onboard compute:**
  - Raspberry Pi 5 (8 GB) — ROS 2 Jazzy navigation stack (Ubuntu 24.04)
  - Jetson Nano (original) — vision / AI waste detection
- **Primary sensors:** IMU + GPS
  - Wheel encoders are **not used for localization** — sand causes excessive slip and drift

## Localization: Two EKF Options

Both options fuse IMU + GPS to produce a filtered odometry estimate. They can be tested and compared independently.

| | Option A: Custom EKF | Option B: robot\_localization |
|---|---|---|
| Package | `beach_robot_custom_ekf` | `beach_robot_localization` |
| Implementation | Hand-written 5-state EKF in C++ | `robot_localization` ekf\_node + navsat\_transform\_node |
| State | \[x, y, theta, v, omega\] | Full 15-state (position, orientation, velocity, acceleration) |
| IMU handling | Accel/gyro as control inputs in process model | Configurable per-axis fusion |
| GPS handling | Equirectangular projection to local frame | navsat\_transform\_node (UTM conversion) |
| Tuning | Direct Q/R matrices in YAML | Per-sensor covariance config in YAML |
| Best for | Learning, full control over filter behavior | Production use, well-tested, handles edge cases |

## Prerequisites

### Host machine

- Ubuntu 22.04 (tested) or any Linux with Docker support
- Docker Engine and Docker Compose v2
- NVIDIA GPU + drivers

### NVIDIA Container Toolkit (required)

GPU passthrough is needed for Gazebo rendering. Install on the **host**:

```bash
# Add NVIDIA repo
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Configure Docker runtime and restart
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Verify with:

```bash
docker run --rm --runtime=nvidia --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi
```

## Quick Start

```bash
# Clone / enter the project
cd beach_waste_path_planning

# Allow X11 forwarding for GUI tools
xhost +local:docker

# Build the Docker image (first time takes a few minutes)
cd docker
docker compose build

# Start the container
docker compose up -d

# Enter the container
docker compose exec ros2-dev bash
```

### Inside the container

```bash
# Build all packages
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash

# Test Option A (custom EKF)
ros2 launch beach_robot_custom_ekf ekf_custom.launch.py

# Test Option B (robot_localization)
ros2 launch beach_robot_localization dual_ekf_navsat.launch.py
```

### Open additional terminals

```bash
# From the host, open more shells into the running container
cd beach_waste_path_planning/docker
docker compose exec ros2-dev bash
```

### GUI tools

All GUI tools render on the host display via X11 forwarding:

```bash
# Inside the container
rviz2 &
ros2 run plotjuggler plotjuggler &
gz sim &
```

### Stop the container

```bash
cd beach_waste_path_planning/docker
docker compose down
```

## Project Structure

```
beach_waste_path_planning/
├── docker/
│   ├── Dockerfile              # ROS 2 Jazzy + Gazebo Harmonic + dev tools
│   ├── docker-compose.yml      # NVIDIA GPU + X11 + volume mounts
│   └── entrypoint.sh           # Sources ROS 2 and workspace on entry
├── src/
│   ├── beach_robot_custom_ekf/     # Option A: custom EKF (C++)
│   │   ├── src/ekf_imu_gps.cpp
│   │   ├── config/ekf_params.yaml
│   │   └── launch/ekf_custom.launch.py
│   └── beach_robot_localization/   # Option B: robot_localization config
│       ├── config/ekf.yaml
│       ├── config/navsat.yaml
│       └── launch/dual_ekf_navsat.launch.py
├── config/                     # Shared configuration files
├── scripts/                    # Utility scripts
├── .gitignore
├── .dockerignore
└── README.md
```

## Sensor Topics

| Topic | Type | Source | Used by |
|-------|------|--------|---------|
| `/imu/data` | `sensor_msgs/Imu` | IMU driver | Both EKF options |
| `/gps/fix` | `sensor_msgs/NavSatFix` | GPS driver | Both EKF options |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF output | Nav2, downstream consumers |
| `/odometry/gps` | `nav_msgs/Odometry` | navsat\_transform\_node | Option B only (internal) |

## Development Notes

- **Editing code:** edit files in `src/` and `config/` from your host IDE — they are bind-mounted into the container
- **Build cache:** colcon build/install/log directories are stored in Docker named volumes, so rebuilds are fast across container restarts
- **Clearing build cache:** `docker volume rm docker_build_cache docker_install_cache docker_log_cache`
- **Rebuilding the image:** `cd docker && docker compose build --no-cache`
- **UID mapping:** the container user matches host UID 1000 to avoid file permission issues
