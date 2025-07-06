# Use official ROS 2 Humble image with desktop tools
FROM osrf/ros:humble-desktop

# Set bash as default shell and configure environment
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# Install core development tools and project dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros-gz \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user with sudo privileges
RUN useradd -m -G sudo ros && \
    echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/ros

# Set up workspace with proper permissions
RUN mkdir -p /ros2_ws/src && \
    chown -R ros:ros /ros2_ws

USER ros
WORKDIR /ros2_ws

# Configure environment (avoids needing entrypoint.sh during development)
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Copy only necessary files (better than full COPY for dev containers)
COPY --chown=ros:ros package.xml /ros2_ws/src/prm/
# COPY --chown=ros:ros entrypoint.sh /home/ros/

# Install dependencies and build (cached as separate layer)
RUN sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys="gz_ros2_control" && \
    colcon build --symlink-install

# Health check for container monitoring
HEALTHCHECK --interval=30s --timeout=5s \
    CMD ros2 node list || exit 1

# Default command (overridden by devcontainer.json)
# CMD ["/home/ros/entrypoint.sh"]