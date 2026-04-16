FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# ── Core ROS 2 packages ─────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    # Gazebo Harmonic + ROS bridge
    ros-jazzy-ros-gz \
    # Navigation 2
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    # Localization
    ros-jazzy-robot-localization \
    ros-jazzy-slam-toolbox \
    # Robot description tooling
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    # TF / diagnostics
    ros-jazzy-tf2-tools \
    ros-jazzy-tf-transformations \
    ros-jazzy-diagnostic-updater \
    # Controller stack (for diff-drive simulation)
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    # GPS / sensor messages
    ros-jazzy-geographic-msgs \
    ros-jazzy-mavros-msgs \
    && rm -rf /var/lib/apt/lists/*

# ── PlotJuggler (separate layer — may lag behind on new distros) ────
RUN apt-get update && apt-get install -y \
    ros-jazzy-plotjuggler-ros \
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

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'export GZ_SIM_RESOURCE_PATH=/ros2_ws/src:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc

COPY --chown=${USERNAME}:${USERNAME} docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
