#!/bin/bash
# Este script serve como o painel de controle principal para iniciar e parar
# os diferentes componentes da aplicação ROS.

# Garante que o script pare se ocorrer um erro
set -e

# --- Configurações Globais ---
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly BLUE='\033[0;34m'
readonly YELLOW='\033[1;33m'
readonly NC='\033[0m'

# Define o terminal a ser usado para abrir novos processos. 'terminator' é ótimo para layouts.
readonly TERMINAL="terminator"
# Caminho absoluto para a raiz do workspace
readonly WORKSPACE="/home/ros/ros2_ws"
# Nome do pacote ROS que contém os arquivos de launch
readonly LAUNCH_PACKAGE="prm"
# Comando para ativar o ambiente do workspace. Será usado em cada novo terminal.
readonly SOURCE_CMD="source ${WORKSPACE}/install/setup.bash"

# --- Funções ---

# Função para iniciar um processo ROS em um novo terminal.
# Argumentos: 1-Nome do processo, 2-Comando a ser executado, 3-Cor para o log
start_process() {
    local name="$1"
    local command="$2"
    local color="$3"
    
    echo -e "${color}Iniciando ${name}...${NC}"
    # Abre um novo terminal com um título específico e executa o comando.
    # O 'bash' no final mantém o terminal aberto para inspeção após o término do comando.
    ${TERMINAL} -T "${name}" -e "bash -c '${SOURCE_CMD} && ${command}; exec bash'" &
}

# Função para parar todos os processos relacionados à simulação.
stop_processes() {
    echo -e "${RED}Parando todos os processos da simulação...${NC}"
    # Lista de processos a serem finalizados. É mais seguro focar nos processos do ROS e Gazebo.
    # Usar 'pkill -f' permite encontrar processos pelo nome completo do comando.
    local processes_to_kill=(
        "ros2 launch"
        "gz sim" # O executável do Gazebo Fortress é 'gz sim'
        "robot_state_publisher"
        "parameter_bridge"
    )
    
    for proc in "${processes_to_kill[@]}"; do
        if pgrep -f "${proc}" > /dev/null; then
            pkill -f "${proc}"
            echo "Processos contendo '${proc}' finalizados."
        fi
    done
    
    # Mata o terminator por último para garantir que os processos filhos tenham terminado.
    if pgrep -f "terminator" > /dev/null; then
        pkill -f "terminator"
        echo "Janelas do Terminator finalizadas."
    fi

    echo -e "${GREEN}Processos finalizados.${NC}"
}

# Mostra como usar o script.
show_usage() {
    echo -e "${YELLOW}Uso:${NC} $0 [comando]"
    echo -e "\nComandos disponíveis:"
    echo -e "  ${GREEN}sim${NC}          - Inicia a simulação no Gazebo."
    echo -e "  ${BLUE}ctrl${NC}         - Inicia os nós de controle do robô e RViz."
    echo -e "  ${YELLOW}both${NC}         - Inicia a simulação e o controle em sequência."
    echo -e "  ${RED}kill${NC}         - Para todos os processos da simulação."
    echo -e "  ${YELLOW}custom <file>${NC} - Executa um arquivo de launch personalizado (ex: $0 custom meu_teste.launch.py)."
}

# --- Execução Principal ---

# Verifica se o ambiente do workspace está ativado. Se não, exibe um erro e sai.
if [ -z "$AMENT_PREFIX_PATH" ] || [[ "$AMENT_PREFIX_PATH" != *"$WORKSPACE/install"* ]]; then
    echo -e "${RED}Erro: Ambiente do workspace não ativado.${NC}"
    echo -e "Por favor, execute '${YELLOW}source ${WORKSPACE}/install/setup.bash${NC}' antes de usar este script."
    exit 1
fi

# Verifica se algum comando foi passado. Se não, mostra o menu de ajuda.
if [ $# -eq 0 ]; then
    show_usage
    exit 0
fi

# Analisa o primeiro argumento para determinar a ação.
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
