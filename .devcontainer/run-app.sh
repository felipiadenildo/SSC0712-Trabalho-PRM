#!/bin/bash

# Configurações globais
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly BLUE='\033[0;34m'
readonly YELLOW='\033[1;33m'
readonly NC='\033[0m'

readonly TERMINAL="terminator --layout=default -e"
readonly WORKSPACE="/ros2_ws"
readonly LAUNCH_PACKAGE="prm"
readonly SOURCE_CMD="source ${WORKSPACE}/install/setup.bash"

# Funções principais
start_process() {
    local name=$1
    local command=$2
    local color=$3
    local keep_open=${4:-false}
    
    echo -e "${color}Iniciando ${name}...${NC}"
    if [ "$keep_open" = true ]; then
        bash -c "${SOURCE_CMD} && ${command}" &
    else
        ${TERMINAL} "bash -c '${SOURCE_CMD} && ${command}; bash'" &
    fi
}

stop_processes() {
    local processes=("ros2 launch" "gzserver" "terminator")
    
    echo -e "${RED}Parando todos os processos...${NC}"
    for proc in "${processes[@]}"; do
        pkill -f "${proc}" && echo "Processos ${proc} finalizados"
    done
}

show_usage() {
    echo -e "${BLUE}Uso avançado:${NC} $0 [comando] [opções]"
    echo -e "Comandos disponíveis:"
    echo -e "  ${GREEN}sim${NC} [--keep-open]    - Inicia simulação Gazebo"
    echo -e "  ${BLUE}ctrl${NC} [--keep-open]   - Inicia controle do robô"
    echo -e "  ${YELLOW}both${NC} [--keep-open] - Inicia simulação e controle"
    echo -e "  ${RED}kill${NC}               - Para todos os processos"
    echo -e "  ${BLUE}custom${NC} <launcher> - Executa arquivo launch personalizado"
    echo -e "\nOpções:"
    echo -e "  --keep-open    Mantém o terminal atual aberto"
}

# Main execution
source "${WORKSPACE}/install/setup.bash"
colcon build --symlink-install

keep_open=false
custom_launch=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --keep-open)
            keep_open=true
            shift
            ;;
        sim|simulation)
            command="sim"
            shift
            ;;
        ctrl|control)
            command="ctrl"
            shift
            ;;
        both)
            command="both"
            shift
            ;;
        kill|stop)
            command="kill"
            shift
            ;;
        custom)
            command="custom"
            custom_launch="$2"
            shift 2
            ;;
        *)
            show_usage
            exit 1
            ;;
    esac
done

case "$command" in
    sim)
        start_process "Simulação" "ros2 launch ${LAUNCH_PACKAGE} inicia_simulacao.launch.py" "${GREEN}" "$keep_open"
        ;;
    ctrl)
        start_process "Controle" "ros2 launch ${LAUNCH_PACKAGE} carrega_robo.launch.py" "${BLUE}" "$keep_open"
        ;;
    both)
        start_process "Simulação" "ros2 launch ${LAUNCH_PACKAGE} inicia_simulacao.launch.py" "${GREEN}" "$keep_open"
        sleep 3  # Pequeno delay para garantir que a simulação iniciou
        start_process "Controle" "ros2 launch ${LAUNCH_PACKAGE} carrega_robo.launch.py" "${BLUE}" "$keep_open"
        ;;
    custom)
        if [ -z "$custom_launch" ]; then
            echo -e "${RED}Erro: Nenhum arquivo launch especificado${NC}"
            show_usage
            exit 1
        fi
        start_process "Customizado" "ros2 launch ${LAUNCH_PACKAGE} ${custom_launch}" "${YELLOW}" "$keep_open"
        ;;
    kill)
        stop_processes
        ;;
    *)
        show_usage
        exit 1
        ;;
esac