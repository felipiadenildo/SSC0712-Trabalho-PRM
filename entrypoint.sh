#!/bin/bash
set -e

# Configura o ambiente do ROS 2
source /opt/ros/humble/setup.bash
# Configura o ambiente do seu workspace
source /ros2_ws/install/setup.bash

# Executa o comando que foi passado para o contêiner (ex: 'bash', 'ros2 launch ...')
exec "$@"