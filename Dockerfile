# .devcontainer/Dockerfile
ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop-full

# Set up environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=42
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV SHELL=/bin/bash
ENV TERM=xterm-256color

# Create workspace and configure user
RUN mkdir -p /ros2_ws/src && \
    # Create user with proper UID/GID to match host
    useradd -m -u 1000 -G sudo,video ros && \
    echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/ros && \
    # Ensure bash is default shell
    chsh -s /bin/bash ros

# Install essential tools and dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    lsb-release \
    gnupg \
    xterm \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --upgrade \
    pip \
    setuptools \
    wheel \
    flake8 \
    black \
    pytest \
    pytest-cov

# Set up rosdep
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update

# Configure environment for ros user
USER ros
WORKDIR /ros2_ws

# Set up bashrc with proper environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/ros2_ws/src/prm/models:\${GAZEBO_MODEL_PATH}" >> ~/.bashrc && \
    echo "cd /ros2_ws" >> ~/.bashrc && \
    echo "alias build='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo'" >> ~/.bashrc

# Health check for container
HEALTHCHECK --interval=30s --timeout=5s \
    CMD ros2 node list || exit 1

# Default command (overridden by compose/devcontainer)
CMD ["/bin/bash", "-i"]  # Force interactive mode