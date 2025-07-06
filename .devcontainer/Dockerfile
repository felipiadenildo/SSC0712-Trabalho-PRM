ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

# Configuração básica
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV SHELL=/bin/bash

# Instalações essenciais
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    terminator \
    mesa-utils \
    libgl1-mesa-glx \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    dbus-x11 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Configuração do usuário
RUN useradd -m -u 1000 -G sudo,video ros && \
    echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/ros

# Configuração do workspace
USER ros
WORKDIR /ros2_ws

# Configuração do ambiente
RUN mkdir -p ~/.config/terminator && \
    cat > ~/.config/terminator/config <<EOL
[layouts]
  [[default]]
    [[[child1]]]
      type = Terminal
      parent = window0
    [[[window0]]]
      type = Window
      parent = ""
EOL

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/ros2_ws/src/prm/models:\${GAZEBO_MODEL_PATH}" >> ~/.bashrc && \
    echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc && \
    echo "alias build='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo'" >> ~/.bashrc

# Configuração dos scripts
COPY --chown=ros:ros .devcontainer/setup-env.sh .devcontainer/run-app.sh /home/ros/scripts/
RUN sudo chmod +x /home/ros/scripts/*.sh && \
    sudo chown -R ros:ros /home/ros/scripts

ENV PATH="/home/ros/scripts:${PATH}"

# Ponto de entrada padrão
CMD ["/bin/bash"]