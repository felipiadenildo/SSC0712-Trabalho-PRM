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

# Create workspace and non-root user
RUN mkdir -p /ros2_ws/src && \
    useradd -m -G sudo,video ros && \
    echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/ros && \
    chown -R ros:ros /ros2_ws

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
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --upgrade \
    pip \
    setuptools \
    wheel \
    flake8 \
    black \
    pytest

# Set up rosdep
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update

# Switch to non-root user
USER ros
WORKDIR /ros2_ws

# Configure environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/ros2_ws/src/prm/models:\${GAZEBO_MODEL_PATH}" >> ~/.bashrc

# Copy entrypoint script
COPY --chown=ros:ros .devcontainer/ros-setup.sh /home/ros/
RUN sudo chmod +x /home/ros/ros-setup.sh

# Default command (overridden by compose/devcontainer)
CMD ["/bin/bash"]