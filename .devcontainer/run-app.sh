#!/bin/bash
# Painel de Controle Definitivo para o Ambiente de Desenvolvimento ROS 2.
# Este script automatiza a compilação, o lançamento de nós em múltiplos
# terminais e o gerenciamento geral do projeto.

# --- Configurações Globais ---
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly BLUE='\033[0;34m'
readonly YELLOW='\033[1;33m'
readonly NC='\033[0m' # No Color

readonly WORKSPACE="/home/ros/ros2_ws"
readonly SCRIPT_PATH=$(realpath "$0")
readonly SESSION_NAME="prm_session"

# --- Função de Carregamento do Ambiente ---
# Garante que o ambiente ROS 2 e o workspace local estejam carregados.
source_workspace() {
    # Carrega o ambiente base do ROS 2.
    source /opt/ros/humble/setup.bash
    
    # Carrega o nosso workspace local, SE ele já tiver sido compilado.
    if [ -f "${WORKSPACE}/install/setup.bash" ]; then
        source "${WORKSPACE}/install/setup.bash"
    else
        # Não imprime aviso se o comando for 'build', pois é esperado que não esteja compilado.
        if [[ "$1" != "build" ]]; then
            echo -e "${YELLOW}[AVISO]${NC} Workspace não compilado. O ambiente local não foi carregado."
            echo -e "         Execute o comando 'build' primeiro."
        fi
    fi
}

# --- Funções de Gerenciamento de Terminal (com tmux) ---

# Verifica se o tmux está instalado.
check_tmux() {
    if ! command -v tmux &> /dev/null; then
        echo -e "${RED}ERRO: 'tmux' não está instalado, mas é necessário para este script.${NC}"
        echo -e "      Por favor, instale-o (ex: sudo apt-get update && sudo apt-get install tmux) e tente novamente."
        exit 1
    fi
}

# Inicia uma sessão tmux se ela ainda não existir.
start_tmux_session() {
    check_tmux
    tmux has-session -t $SESSION_NAME 2>/dev/null
    if [ $? != 0 ]; then
        echo -e "${GREEN}Iniciando nova sessão tmux chamada '${SESSION_NAME}'...${NC}"
        tmux new-session -d -s $SESSION_NAME
        echo -e "Para ver os terminais, anexe à sessão com o comando: ${YELLOW}tmux attach -t ${SESSION_NAME}${NC}"
    fi
}

# Executa um comando em uma nova janela do tmux.
run_in_new_window() {
    local window_name="$1"
    local command_to_run="$2"

    start_tmux_session
    echo -e "${BLUE}Abrindo nova janela '${window_name}' e executando: ${command_to_run}${NC}"
    
    # Chama o próprio script dentro do tmux para garantir que o ambiente seja carregado corretamente.
    # O 'exec bash' no final mantém o terminal aberto para depuração, mesmo que o comando falhe.
    tmux new-window -t $SESSION_NAME -n "${window_name}" "bash -c '${SCRIPT_PATH} ${command_to_run}; exec bash'"
}

# --- Funções de Lógica (Etapas Individuais) ---

func_build() {
    echo -e "${GREEN}=== Construindo o Workspace ===${NC}"
    cd "${WORKSPACE}"
    source_workspace "build" # Carrega o ambiente base
    colcon build --symlink-install
    if [ $? -ne 0 ]; then
        echo -e "\n${RED}ERRO: A compilação falhou.${NC}"
        exit 1
    fi
    echo -e "\n${GREEN}Construção finalizada com sucesso!${NC}"
}

func_start_simulation() {
    run_in_new_window "GazeboSim" "launch prm inicia_simulacao.launch.py world:=arena_cilindros.sdf"
}

func_load_robot() {
    run_in_new_window "RobotCtrl" "launch prm carrega_robo.launch.py"
}

func_start_vision() {
    run_in_new_window "Vision" "run prm vision_node"
}

func_start_mapper() {
    run_in_new_window "Mapper" "run prm robo_mapper"
}

func_start_planner() {
    run_in_new_window "Planner" "run prm path_planner_node"
}

func_start_fsm() {
    run_in_new_window "StateMach" "run prm state_machine_node"
}

func_run_all() {
    echo -e "${YELLOW}=== Iniciando todos os componentes da aplicação ===${NC}"
    func_build
    echo -e "${GREEN}Build concluído. Iniciando nós em 2 segundos...${NC}"
    sleep 5
    func_start_simulation
    echo "Aguardando o Gazebo iniciar (5s)..."
    sleep 5
    func_load_robot
    sleep 5
    func_start_vision
    sleep 3
    func_start_mapper
    sleep 3
    func_start_planner
    sleep 3
    func_start_fsm
    echo -e "\n${GREEN}Todos os nós foram iniciados em janelas separadas do tmux.${NC}"
    echo -e "Anexe à sessão para visualizar: ${YELLOW}tmux attach -t ${SESSION_NAME}${NC}"
    echo -e "Para parar tudo, use o comando: ${RED}${0} kill-all${NC}"
    # tmux attach -t prm_session
}

func_kill_all() {
    check_tmux
    tmux has-session -t $SESSION_NAME 2>/dev/null
    if [ $? == 0 ]; then
        echo -e "${RED}Matando a sessão tmux '${SESSION_NAME}' e todos os seus processos...${NC}"
        tmux kill-session -t $SESSION_NAME
        echo -e "${GREEN}Sessão finalizada.${NC}"
    else
        echo -e "${YELLOW}Nenhuma sessão tmux chamada '${SESSION_NAME}' encontrada.${NC}"
    fi
}

# Delega qualquer comando 'ros2' (launch, run, topic, etc.)
func_ros2_command() {
    echo -e "${BLUE}=== Executando Comando ROS 2: ros2 $@ ===${NC}"
    source_workspace
    exec ros2 "$@"
}

func_usage() {
    echo -e "\n${YELLOW}Painel de Controle do Projeto PRM - ROS 2${NC}"
    echo "----------------------------------------------------"
    echo "Uso: $0 <comando> [argumentos...]"
    echo ""
    echo "Comandos de Automação:"
    echo -e "  ${GREEN}build${NC}                - Compila o workspace do projeto."
    echo -e "  ${GREEN}start-sim${NC}            - Inicia a simulação Gazebo em um novo terminal."
    echo -e "  ${GREEN}load-robot${NC}           - Carrega os controladores do robô em um novo terminal."
    echo -e "  ${GREEN}start-vision${NC}          - Inicia o nó de visão em um novo terminal."
    echo -e "  ${GREEN}start-mapper${NC}          - Inicia o nó de mapeamento em um novo terminal."
    echo -e "  ${GREEN}start-planner${NC}         - Inicia o nó de planejamento de caminho em um novo terminal."
    echo -e "  ${GREEN}start-fsm${NC}             - Inicia a máquina de estados em um novo terminal."
    echo -e "  ${YELLOW}run-all${NC}              - Compila e executa todos os passos acima em ordem."
    echo -e "  ${RED}kill-all${NC}             - Mata a sessão tmux e todos os processos iniciados."
    echo ""
    echo "Comandos ROS 2 (executados diretamente):"
    echo "  Qualquer outro comando será passado para 'ros2' (ex: launch, run, topic)."
    echo -e "  Exemplo: ${BLUE}$0 launch prm teste_urdf.launch.py${NC}"
    echo "----------------------------------------------------"
    echo -e "NOTA: Os comandos de automação usam ${YELLOW}tmux${NC} para gerenciar terminais."
    echo -e "      Use '${YELLOW}tmux attach -t ${SESSION_NAME}${NC}' para ver os terminais."
}

# --- Roteador Principal de Comandos ---
if [ $# -eq 0 ]; then
    func_usage
    exit 0
fi

main_command="$1"
shift # Remove o primeiro argumento, o resto será passado para o ros2

case "$main_command" in
    build)
        func_build
        ;;
    start-sim)
        func_start_simulation
        ;;
    load-robot)
        func_load_robot
        ;;
    start-vision)
        func_start_vision
        ;;
    start-mapper)
        func_start_mapper
        ;;
    start-planner)
        func_start_planner
        ;;
    start-fsm)
        func_start_fsm
        ;;
    run-all)
        func_run_all
        ;;
    kill-all)
        func_kill_all
        ;;
    help|--help|-h)
        func_usage
        ;;
    # Qualquer outro comando (launch, run, topic, etc.) será tratado aqui
    *)
        # Re-adiciona o comando original para a lista de argumentos
        set -- "$main_command" "$@"
        func_ros2_command "$@"
        ;;
esac
