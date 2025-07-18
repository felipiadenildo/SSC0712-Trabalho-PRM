#!/bin/bash
# Este script serve como o painel de controle principal para iniciar e parar
# os diferentes componentes da aplicação ROS.
set -e

# --- Configurações Globais ---
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly BLUE='\033[0;34m'
readonly YELLOW='\033[1;33m'
readonly NC='\033[0m' # No Color

readonly WORKSPACE="/home/ros/ros2_ws"
readonly LAUNCH_PACKAGE="prm"

source install/setup.bash

# --- Funções ---

# IMPRIME o comando exato que o usuário deve executar em um novo terminal.
# Esta abordagem é 100% confiável, pois evita problemas de interpretação de shell.
print_command_to_run() {
    local name="$1"
    local ros_command="$2"
    local color="$3"
    
    # Constrói o comando completo que garante que o ambiente seja carregado corretamente.
    local full_command="source /opt/ros/humble/setup.bash && source ${WORKSPACE}/install/setup.bash && ${ros_command}"

    echo -e "${color}--- Para iniciar o ${name} ---${NC}"
    echo -e "1. Abra um NOVO terminal no VS Code (clique no ícone '+' ao lado do nome do terminal)."
    echo -e "2. Copie o comando abaixo, cole no novo terminal e pressione Enter:"
    echo -e "${YELLOW}${full_command}${NC}\n"
}

# Para todos os processos relacionados à simulação.
stop_processes() {
    echo -e "${RED}Parando todos os processos...${NC}"
    # O '|| true' evita que o script pare se um processo não for encontrado.
    pkill -f "ros2 launch" || true
    pkill -f "gz sim" || true
    echo -e "${GREEN}Processos finalizados.${NC}"
}

# Mostra como usar o script.
show_usage() {
    echo -e "${YELLOW}Uso:${NC} runapp [comando]"
    echo -e "\nComandos disponíveis:"
    echo -e "  ${GREEN}sim${NC}          - Mostra o comando para iniciar a simulação."
    echo -e "  ${BLUE}ctrl${NC}         - Mostra o comando para iniciar o controle do robô."
    echo -e "  ${RED}kill${NC}         - Para todos os processos ROS e Gazebo."
}

# --- Execução Principal ---

# Verifica se o workspace foi compilado.
if [ ! -f "${WORKSPACE}/install/setup.bash" ]; then
    echo -e "${RED}Erro: O workspace ainda não foi compilado.${NC}"
    echo -e "Execute o comando '${YELLOW}build${NC}' primeiro."
    exit 1
fi

if [ $# -eq 0 ]; then
    show_usage
    exit 0
fi

case "$1" in
    sim|simulation)
        print_command_to_run "Simulação (Gazebo)" "ros2 launch ${LAUNCH_PACKAGE} inicia_simulacao.launch.py" "${GREEN}"
        ;;
    ctrl|control)
        print_command_to_run "Controle do Robô (RViz, etc.)" "ros2 launch ${LAUNCH_PACKAGE} carrega_robo.launch.py" "${BLUE}"
        ;;
    kill|stop)
        stop_processes
        ;;
    *)
        echo -e "${RED}Erro: Comando '$1' desconhecido.${NC}"; show_usage; exit 1
        ;;
esac
