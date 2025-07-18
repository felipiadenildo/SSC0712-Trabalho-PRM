#!/bin/bash
# Painel de Controle Definitivo para o Ambiente de Desenvolvimento ROS 2.
# Garante que o ambiente ROS seja carregado antes de qualquer comando.

# --- Configurações Globais ---
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly BLUE='\033[0;34m'
readonly YELLOW='\033[1;33m'
readonly NC='\033[0m' # No Color

readonly WORKSPACE="/home/ros/ros2_ws"

# --- Função de Carregamento do Ambiente ---
# Esta é a correção mais importante. Esta função será chamada antes de executar
# qualquer outro comando do ROS.
source_workspace() {
    # Carrega o ambiente base do ROS 2.
    source /opt/ros/humble/setup.bash
    
    # Carrega o nosso workspace local, SE ele já tiver sido compilado.
    if [ -f "${WORKSPACE}/install/setup.bash" ]; then
        source "${WORKSPACE}/install/setup.bash"
    else
        echo -e "${YELLOW}[AVISO]${NC} Workspace não compilado. O ambiente local não foi carregado."
        echo -e "         Execute o comando 'build' primeiro."
    fi
}

# --- Funções de Lógica ---
func_build() {
    echo -e "${GREEN}=== Construindo o Workspace ===${NC}"
    cd "${WORKSPACE}"
    colcon build --symlink-install
    if [ $? -ne 0 ]; then
        echo -e "\n${RED}ERRO: A compilação falhou.${NC}"
        exit 1
    fi
    echo -e "\n${GREEN}Construção finalizada com sucesso!${NC}"
}

# Delega qualquer comando 'ros2' (launch, run, topic, etc.)
func_ros2_command() {
    echo -e "${BLUE}=== Executando Comando ROS 2: ros2 $@ ===${NC}"
    # Carrega o ambiente e depois executa o comando passado.
    source_workspace
    exec ros2 "$@"
}

func_usage() {
    echo -e "\n${YELLOW}Painel de Controle do Projeto PRM - ROS 2${NC}"
    echo "----------------------------------------"
    echo "Este script agora é um 'wrapper' para os comandos ROS 2."
    echo ""
    echo "Uso: run-app.sh <comando>"
    echo ""
    echo "Comandos de Gerenciamento:"
    echo -e "  ${GREEN}build${NC}        - Compila o workspace do projeto."
    echo ""
    echo "Comandos ROS 2 (Exemplos):"
    echo -e "  ${BLUE}launch prm carrega_robo.py${NC}"
    echo -e "  ${BLUE}run prm vision_node${NC}"
    echo -e "  ${BLUE}topic echo /vision/flag_position${NC}"
    echo "----------------------------------------"
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
    # Qualquer outro comando (launch, run, topic, etc.) será tratado aqui
    *)
        # Re-adiciona o comando original para a lista de argumentos
        set -- "$main_command" "$@"
        func_ros2_command "$@"
        ;;
esac