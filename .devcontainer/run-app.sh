#!/bin/bash

# Configurações globais
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m'

readonly TERMINAL="terminator --layout=grid -e"
readonly WORKSPACE="/ros2_ws"
readonly LAUNCH_PACKAGE="prm"
readonly SOURCE_CMD="source ${WORKSPACE}/install/setup.bash"

# Funções principais
start_process() {
    local name=$1
    local command=$2
    local color=$3
    
    echo -e "${color}Iniciando ${name}...${NC}"
    ${TERMINAL} "bash -c '${SOURCE_CMD} && ${command}; bash'" &
}

stop_processes() {
    local processes=("ros2 launch" "gzserver" "terminator")
    
    echo -e "${RED}Parando todos os processos...${NC}"
    for proc in "${processes[@]}"; do
        pkill -f "${proc}" && echo "Processos ${proc} finalizados"
    done
}

show_usage() {
    echo -e "${BLUE}Uso:${NC} $0 [comando]"
    echo -e "Comandos disponíveis:"
    echo -e "  ${GREEN}sim${NC}     - Inicia simulação Gazebo"
    echo -e "  ${BLUE}ctrl${NC}    - Inicia controle do robô"
    echo -e "  ${RED}kill${NC}    - Para todos os processos"
}

# Main execution
source "${WORKSPACE}/install/setup.bash"

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
        show_usage
        exit 1
        ;;
esac