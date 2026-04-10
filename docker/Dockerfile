FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ── Core ROS 2 packages ─────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    # Gazebo Fortress + ROS bridge
    ros-humble-ros-gz \
    # Navigation 2
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # Localization
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    # Robot description tooling
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    # TF / diagnostics
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-diagnostic-updater \
    # Controller stack (for diff-drive simulation)
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ign-ros2-control \
    # GPS / sensor messages
    ros-humble-geographic-msgs \
    ros-humble-mavros-msgs \
    && rm -rf /var/lib/apt/lists/*

# ── PlotJuggler (separate layer — may lag behind on new distros) ────
RUN apt-get update && apt-get install -y \
    ros-humble-plotjuggler-ros \
    || echo "plotjuggler-ros not available as deb, will install from source if needed" \
    && rm -rf /var/lib/apt/lists/*

# ── Dev tools ────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    libeigen3-dev \
    libyaml-cpp-dev \
    nano \
    vim \
    htop \
    wget \
    curl \
    gdb \
    bash-completion \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# ── Non-root user (matches typical host UID 1000) ───────────────────
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=devuser

# Reuse existing UID/GID if already taken, otherwise create fresh
RUN EXISTING_USER=$(getent passwd ${USER_UID} | cut -d: -f1) && \
    if [ -n "$EXISTING_USER" ] && [ "$EXISTING_USER" != "${USERNAME}" ]; then \
        usermod -l ${USERNAME} -d /home/${USERNAME} -m $EXISTING_USER && \
        groupmod -n ${USERNAME} $(getent group ${USER_GID} | cut -d: -f1) 2>/dev/null || true; \
    elif [ -z "$EXISTING_USER" ]; then \
        groupadd --gid ${USER_GID} ${USERNAME} 2>/dev/null || true && \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}; \
    fi && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# ── Workspace setup ─────────────────────────────────────────────────
RUN mkdir -p /ros2_ws/src && chown -R ${USERNAME}:${USERNAME} /ros2_ws

# ── Shell config for the dev user ────────────────────────────────────
USER ${USERNAME}
WORKDIR /ros2_ws

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'export IGN_GAZEBO_RESOURCE_PATH=/ros2_ws/src:${IGN_GAZEBO_RESOURCE_PATH}' >> ~/.bashrc

COPY --chown=${USERNAME}:${USERNAME} docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
