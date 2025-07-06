#!/bin/bash

# Cores
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configurações
TERMINAL="terminator --layout=grid -e"
WORKSPACE="/ros2_ws"
LAUNCH_PACKAGE="prm"

start_process() {
    local name=$1
    local command=$2
    local color=$3
    
    echo -e "${color}Iniciando ${name}...${NC}"
    $TERMINAL "bash -c 'source ${WORKSPACE}/install/setup.bash && ${command}; bash'" &
}

stop_processes() {
    echo -e "${RED}Parando todos os processos...${NC}"
    pkill -f "ros2 launch" && echo "Processos ROS finalizados"
    pkill -f gzserver && echo "Processos Gazebo finalizados"
    pkill -f terminator && echo "Terminais fechados"
}

case "$1" in
    sim|simulation)
        start_process "Simulação" "ros2 launch ${LAUNCH_PACKAGE} inicia_simulacao.launch.py" "${GREEN}"
        ;;
    ctrl|control)
        start_process "Controle" "ros2 launch ${LAUNCH_PACKAGE} carrega_robo.launch.py" "${BLUE}"
        ;;
    kill|stop)
        stop_processes
        ;;
    *)
        echo -e "${BLUE}Uso:${NC} $0 [comando]"
        echo -e "Comandos disponíveis:"
        echo -e "  ${GREEN}sim${NC}     - Inicia simulação Gazebo"
        echo -e "  ${BLUE}ctrl${NC}    - Inicia controle do robô"
        echo -e "  ${RED}kill${NC}    - Para todos os processos"
        ;;
esac