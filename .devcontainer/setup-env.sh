#!/bin/bash
set -eo pipefail

# Cores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'


# --- Configurações de Permissões ---
sudo chown -R ros:ros /ros2_ws 2>/dev/null || true

# --- Funções Auxiliares ---
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

check_gpu() {
    # Configurações de X11 e DBUS
    export DISPLAY=$(grep -m 1 nameserver /etc/resolv.conf | awk '{print $2}'):0
    export XDG_RUNTIME_DIR=/run/user/$(id -u)
    export DBUS_SESSION_BUS_ADDRESS=unix:path=$XDG_RUNTIME_DIR/bus

    if command -v nvidia-smi &>/dev/null; then
        log_info "GPU NVIDIA detectada:"
        nvidia-smi --query-gpu=name --format=csv,noheader
    else
        log_warn "GPU NVIDIA não detectada - usando renderização por software"
        export LIBGL_ALWAYS_SOFTWARE=1
        export GALLIUM_DRIVER=llvmpipe
    fi
}

setup_workspace() {
    if [ -f "/ros2_ws/install/setup.bash" ]; then
        source /ros2_ws/install/setup.bash
    else
        log_info "Construindo workspace..."
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    fi
}

sudo rosdep init
rosdep update

# --- Execução Principal ---
check_gpu

# Configuração do ROS
source /opt/ros/humble/setup.bash

setup_workspace

log_info "Ambiente ROS 2 configurado com sucesso!"
echo -e "Comandos disponíveis:"
echo -e "  ${YELLOW}run-app.sh sim${NC}    - Inicia simulação"
echo -e "  ${YELLOW}run-app.sh ctrl${NC}   - Inicia controle"
echo -e "  ${YELLOW}run-app.sh kill${NC}   - Para todos os processos"

echo -e "\n# Alias para run-app.sh" >> ~/.bashrc
echo 'alias rosapp="/ros2_ws/src/prm/.devcontainer/run-app.sh"' >> ~/.bashrc
source ~/.bashrc  