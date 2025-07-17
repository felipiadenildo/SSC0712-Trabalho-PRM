#!/bin/bash
# Este script serve como o painel de controle principal para iniciar e parar
# os diferentes componentes da aplicação ROS.
set -e

# --- Configurações Globais ---
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly BLUE='\033[0;34m'
readonly YELLOW='\033[1;33m'
readonly NC='\033[0m'

readonly WORKSPACE="/home/ros/ros2_ws"
readonly LAUNCH_PACKAGE="prm"

# --- Funções ---

start_process() {
    local name="$1"
    local command="$2"
    local color="$3"
    
    echo -e "${color}Iniciando ${name}...${NC}"
    # CORREÇÃO DEFINITIVA: Força a execução do .bashrc antes de qualquer outro comando.
    # Isso garante que o ambiente ROS, o workspace local e todos os aliases sejam carregados.
    terminator -T "${name}" -e "bash -c 'source ~/.bashrc && ${command}; exec bash'" &
}

stop_processes() {
    echo -e "${RED}Parando todos os processos da simulação...${NC}"
    local processes_to_kill=(
        "ros2 launch"
        "gz sim"
        "robot_state_publisher"
        "parameter_bridge"
    )
    
    for proc in "${processes_to_kill[@]}"; do
        if pgrep -f "${proc}" > /dev/null; then
            pkill -f "${proc}"
            echo "Processos contendo '${proc}' finalizados."
        fi
    done
    
    if pgrep -f "terminator" > /dev/null; then
        pkill -f "terminator"
        echo "Janelas do Terminator finalizadas."
    fi

    echo -e "${GREEN}Processos finalizados.${NC}"
}

show_usage() {
    echo -e "${YELLOW}Uso:${NC} $0 [comando]"
    echo -e "\nComandos disponíveis:"
    echo -e "  ${GREEN}sim${NC}          - Inicia a simulação no Gazebo."
    echo -e "  ${BLUE}ctrl${NC}         - Inicia os nós de controle do robô e RViz."
    echo -e "  ${YELLOW}both${NC}         - Inicia a simulação e o controle em sequência."
    echo -e "  ${RED}kill${NC}         - Para todos os processos da simulação."
    echo -e "  ${YELLOW}custom <file>${NC} - Executa um arquivo de launch personalizado."
}

# --- Execução Principal ---

# Verifica se o ambiente do workspace já foi compilado.
if [ ! -f "${WORKSPACE}/install/setup.bash" ]; then
    echo -e "${RED}Erro: O workspace ainda não foi compilado.${NC}"
    echo -e "Por favor, execute o comando '${YELLOW}build${NC}' no terminal primeiro."
    exit 1
fi

if [ $# -eq 0 ]; then
    show_usage
    exit 0
fi

case "$1" in
    sim|simulation)
        start_process "Simulação (Gazebo)" "ros2 launch ${LAUNCH_PACKAGE} inicia_simulacao.launch.py" "${GREEN}"
        ;;
    ctrl|control)
        start_process "Controle do Robô (RViz, etc.)" "ros2 launch ${LAUNCH_PACKAGE} carrega_robo.launch.py" "${BLUE}"
        ;;
    both)
        start_process "Simulação (Gazebo)" "ros2 launch ${LAUNCH_PACKAGE} inicia_simulacao.launch.py" "${GREEN}"
        echo "Aguardando 5 segundos para o Gazebo inicializar..."
        sleep 5
        start_process "Controle do Robô (RViz, etc.)" "ros2 launch ${LAUNCH_PACKAGE} carrega_robo.launch.py" "${BLUE}"
        ;;
    custom)
        if [ -z "$2" ]; then
            echo -e "${RED}Erro: Nome do arquivo de launch não especificado.${NC}"
            show_usage
            exit 1
        fi
        start_process "Launch Personalizado" "ros2 launch ${LAUNCH_PACKAGE} $2" "${YELLOW}"
        ;;
    kill|stop)
        stop_processes
        ;;
    *)
        echo -e "${RED}Erro: Comando '$1' desconhecido.${NC}"
        show_usage
        exit 1
        ;;
esac