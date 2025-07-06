#!/bin/bash
set -eo pipefail

# Cores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

check_gpu() {
    if command -v nvidia-smi &>/dev/null; then
        log_info "GPU NVIDIA detectada:"
        nvidia-smi --query-gpu=name --format=csv,noheader
    else
        echo -e "${YELLOW}[WARN]${NC} GPU NVIDIA não detectada - usando renderização por software"
        export LIBGL_ALWAYS_SOFTWARE=1
    fi
}

setup_workspace() {
    if [ ! -f "/ros2_ws/install/setup.bash" ]; then
        log_info "Construindo workspace..."
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    fi
}

# Main execution
check_gpu
setup_workspace

source /opt/ros/$ROS_DISTRO/setup.bash
source /ros2_ws/install/setup.bash

log_info "Ambiente ROS 2 configurado com sucesso!"
echo -e "Comandos disponíveis:"
echo -e "  ${YELLOW}run-app.sh sim${NC}    - Inicia simulação"
echo -e "  ${YELLOW}run-app.sh ctrl${NC}   - Inicia controle"
echo -e "  ${YELLOW}run-app.sh kill${NC}   - Para todos os processos"