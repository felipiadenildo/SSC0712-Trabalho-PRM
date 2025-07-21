#!/bin/bash
# Painel de Controle Definitivo para o Ambiente de Desenvolvimento ROS 2.
# Este script automatiza a compilação, o lançamento de nós e o gerenciamento geral do projeto.

# --- Configurações Globais ---
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly BLUE='\033[0;34m'
readonly YELLOW='\033[1;33m'
readonly NC='\033[0m' # No Color

readonly WORKSPACE="/home/ros/ros2_ws"
readonly SCRIPT_PATH=$(realpath "$0")

# --- Função de Carregamento do Ambiente ---
source_workspace() {
    if [ -f "/opt/ros/humble/setup.bash" ]; then source /opt/ros/humble/setup.bash; fi
    if [ -f "${WORKSPACE}/install/setup.bash" ]; then source "${WORKSPACE}/install/setup.bash"; fi
}

# --- Funções de Lógica (Comandos Principais) ---

func_build() {
    echo -e "${GREEN}=== CONSTRUINDO O WORKSPACE ===${NC}"
    cd "${WORKSPACE}"
    source_workspace "build"
    colcon build --symlink-install --packages-select prm
    if [ $? -ne 0 ]; then
        echo -e "\n${RED}ERRO: A compilação falhou.${NC}"; exit 1;
    fi
    echo -e "\n${GREEN}Construção finalizada com sucesso!${NC}"
}

# --- Funções de Lançamento em Etapas (com salvaguardas) ---

func_start_sim() {
    echo -e "${YELLOW}--- A iniciar a Etapa 1: Simulação (Gazebo) ---${NC}"
    echo -e "${BLUE}A executar limpeza prévia para garantir um ambiente limpo...${NC}"
    func_kill_all # Executa a limpeza sem pedir confirmação

    echo -e "\n${GREEN}A iniciar o Gazebo em segundo plano...${NC}"
    ros2 launch prm inicia_simulacao.launch.py &
    echo -e "Comando para iniciar o Gazebo enviado. Aguarde a janela abrir."
}

func_load_robot() {
    echo -e "${YELLOW}--- A iniciar a Etapa 2: Robô e Controladores ---${NC}"
    ros2 launch prm carrega_robo.launch.py &
    echo -e "${GREEN}Comando para carregar o robô e RViz enviado.${NC}"
}

func_start_logic() {
    echo -e "${YELLOW}--- A iniciar a Etapa 3: Nós de Controlo e Inteligência ---${NC}"
    ros2 run prm vision_node &
    echo -e "${BLUE}--> Nó de Visão iniciado.${NC}"
    sleep 1
    ros2 run prm path_planner_node &
    echo -e "${BLUE}--> Nó de Planeamento de Caminho iniciado.${NC}"
    sleep 1
    ros2 run prm state_machine_node &
    echo -e "${BLUE}--> Nó da Máquina de Estados iniciado.${NC}"
    echo -e "${GREEN}Todos os nós de controlo estão em execução.${NC}"
}

func_run_all() {
    echo -e "${YELLOW}=== INICIANDO SISTEMA COMPLETO EM ETAPAS ===${NC}"
    
    # 1. Limpeza Prévia (Salvaguarda Essencial)
    echo -e "\n${BLUE}Passo 0: A garantir que todos os processos antigos estão encerrados...${NC}"
    func_kill_all
    sleep 2 # Pequena pausa para garantir que os processos terminaram

    # 2. Compilar
    echo -e "\n${BLUE}Passo 1: A compilar o projeto...${NC}"
    func_build
    if [ $? -ne 0 ]; then return; fi

    # 3. Iniciar Simulação
    echo -e "\n${BLUE}Passo 2: A iniciar a simulação...${NC}"
    func_start_sim
    echo "Aguardando 15 segundos para o Gazebo carregar completamente..."
    sleep 15

    # 4. Carregar o Robô
    echo -e "\n${BLUE}Passo 3: A carregar o robô no ambiente...${NC}"
    func_load_robot
    echo "Aguardando 10 segundos para os controladores do robô iniciarem..."
    sleep 10
    
    # 5. Iniciar a Lógica
    echo -e "\n${BLUE}Passo 4: A iniciar a inteligência do robô...${NC}"
    func_start_logic
    
    echo -e "\n\n${YELLOW}*** O SISTEMA COMPLETO ESTÁ EM EXECUÇÃO! ***${NC}"
    echo -e "Observe o Gazebo e o RViz. O robô deve começar a sua missão."
    echo -e "Para parar tudo, execute o comando: ${RED}${0} kill-all${NC}"
}

# --- FUNÇÃO DE ENCERRAMENTO (MELHORADA) ---
# func_kill_all() {
#     read -p "$(echo -e ${RED}'Tem a certeza que quer encerrar TODOS os processos do ROS e Gazebo? (s/n) '${NC})" -n 1 -r
#     echo
#     if [[ $REPLY =~ ^[Ss]$ ]]
#     then
#         func_kill_all_silent
#     else
#         echo -e "${GREEN}Operação de encerramento cancelada.${NC}"
#     fi
# }

func_kill_all() {
    echo -e "${RED}=== INICIANDO PROCEDIMENTO DE ENCERRAMENTO TOTAL ===${NC}"
    
    # 1. Finaliza todos os processos do Gazebo
    pkill -9 -f "ign gazebo" && echo -e "${RED}--> Processos do Gazebo finalizados.${NC}" || true
    
    # 2. Finaliza os nós padrão do ROS
    pkill -9 -f "rviz2" && echo -e "${RED}--> Processo 'rviz2' finalizado.${NC}" || true
    pkill -9 -f "robot_state_publisher" && echo -e "${RED}--> Nó 'robot_state_publisher' finalizado.${NC}" || true
    
    # 3. Finaliza a ponte de comunicação
    pkill -9 -f "parameter_bridge" && echo -e "${RED}--> Ponte 'ros_gz_bridge' finalizada.${NC}" || true

    # 4. Finaliza os processos de controle (A CORREÇÃO PRINCIPAL ESTÁ AQUI)
    #    Esta linha busca pelo comando exato que vimos na sua imagem.
    pkill -9 -f "ros2 control load_controller" && echo -e "${RED}--> Processos 'ros2 control' finalizados.${NC}" || true
    pkill -9 -f "ros2 launch prm carrega_robo" && echo -e "${RED}--> Processos 'ros2 launch' finalizados.${NC}" || true
    #    Mantemos a busca por "spawner" para o caso de você alternar entre as versões do launch file.
    pkill -9 -f "spawner" && echo -e "${RED}--> Spawners de controladores finalizados.${NC}" || true
    
    # 5. Mata os nós específicos do seu projeto
    pkill -9 -f "vision_node" && echo -e "${RED}--> Nó 'vision_node' finalizado.${NC}" || true
    pkill -9 -f "robo_mapper" && echo -e "${RED}--> Nó 'robo_mapper' finalizado.${NC}" || true
    pkill -9 -f "path_planner_node" && echo -e "${RED}--> Nó 'path_planner_node' finalizado.${NC}" || true
    pkill -9 -f "state_machine_node" && echo -e "${RED}--> Nó 'state_machine_node' finalizado.${NC}" || true
    pkill -9 -f "ground_truth_odometry" && echo -e "${RED}--> Nó 'ground_truth_odometry' finalizado.${NC}" || true
    
    # 6. Força o encerramento do daemon do ROS 2
    pkill -9 -f "ros2-daemon" && echo -e "${RED}--> Daemon do ROS 2 finalizado.${NC}" || true

    echo -e "\n${GREEN}=== PROCEDIMENTO DE ENCERRAMENTO CONCLUÍDO ===${NC}"
}

# Delega qualquer comando 'ros2'
func_ros2_command() {
    source_workspace
    exec ros2 "$@"
}

func_usage() {
    echo -e "\n${YELLOW}Painel de Controle do Projeto PRM - ROS 2${NC}"
    echo "----------------------------------------------------"
    echo "Uso: $0 <comando>"
    echo ""
    echo "Comandos de Automação:"
    echo -e "  ${GREEN}build${NC}                - Compila o workspace do projeto."
    echo -e "  ${GREEN}start-sim${NC}            - ETAPA 1: Inicia a simulação Gazebo (limpa processos antigos antes)."
    echo -e "  ${GREEN}load-robot${NC}           - ETAPA 2: Carrega o robô, RViz e controladores."
    echo -e "  ${GREEN}start-logic${NC}          - ETAPA 3: Inicia os nós de inteligência."
    echo -e "  ${YELLOW}run-all${NC}              - Executa todos os passos acima em ordem com pausas."
    echo -e "  ${RED}kill-all${NC}             - Mata TODOS os processos do ROS e Gazebo."
    echo ""
    echo "Comandos ROS 2 (executados diretamente):"
    echo "  Qualquer outro comando será passado para 'ros2'."
    echo -e "  Exemplo: ${BLUE}$0 topic list${NC}"
    echo "----------------------------------------------------"
}

# --- Roteador Principal de Comandos ---
if [ $# -eq 0 ]; then
    func_usage; exit 0;
fi

main_command="$1"
shift

case "$main_command" in
    build) func_build ;;
    start-sim) func_start_sim ;;
    load-robot) func_load_robot ;;
    start-logic) func_start_logic ;;
    run-all) func_run_all ;;
    kill-all) func_kill_all ;;
    help|--help|-h) func_usage ;;
    *)
        set -- "$main_command" "$@"
        func_ros2_command "$@" ;;
esac
