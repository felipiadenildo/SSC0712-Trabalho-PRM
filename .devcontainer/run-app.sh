#!/bin/bash
# Painel de Controle Avançado para o Ambiente de Desenvolvimento ROS 2.
# Este script automatiza a compilação, atualização, execução e finalização
# de todos os componentes do projeto, utilizando o 'terminator' para
# organizar os processos em abas de terminal separadas.

# --- Configurações Globais ---
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly BLUE='\033[0;34m'
readonly YELLOW='\033[1;33m'
readonly NC='\033[0m' # No Color

readonly WORKSPACE="/home/ros/ros2_ws"
readonly PACKAGE_NAME="prm"

# --- Funções de Lógica ---

# COMPILA o workspace do ROS 2.
func_build() {
    echo -e "${GREEN}=== Construindo o Workspace (${PACKAGE_NAME}) ===${NC}"
    cd "${WORKSPACE}"
    colcon build --symlink-install --packages-select "${PACKAGE_NAME}"
    echo -e "${GREEN}Construção finalizada com sucesso!${NC}"
}

# ATUALIZA e instala as dependências do projeto.
func_update_deps() {
    echo -e "${YELLOW}=== Atualizando Dependências (rosdep) ===${NC}"
    cd "${WORKSPACE}"
    source /opt/ros/humble/setup.bash
    rosdep install --from-paths src -i -y --ignore-src
    echo -e "${GREEN}Dependências atualizadas!${NC}"
}

# INICIA a aplicação completa em um layout de terminais separados.
func_start() {
    if [ ! -f "${WORKSPACE}/install/setup.bash" ]; then
        echo -e "${RED}Erro: O workspace ainda não foi compilado.${NC}"
        echo -e "Execute o comando '${YELLOW}runapp build${NC}' primeiro."
        exit 1
    fi

    echo -e "${GREEN}🚀 Iniciando aplicação completa em abas do Terminator...${NC}"

    local source_cmd="source /opt/ros/humble/setup.bash && source ${WORKSPACE}/install/setup.bash"

    # ======================= INÍCIO DA CORREÇÃO =======================
    # Voltando a usar '--new-tab', que é compatível com a versão do Terminator instalada.
    # O erro gráfico que causava problemas com este método já foi corrigido no Dockerfile.
    terminator \
        -T "Aba 1: Simulação (Gazebo)" -e "bash -c '${source_cmd}; ros2 launch ${PACKAGE_NAME} inicia_simulacao.launch.py; exec bash'" \
        --new-tab \
        -T "Aba 2: Planejador de Caminhos" -e "bash -c '${source_cmd}; sleep 8; ros2 run ${PACKAGE_NAME} path_planner_node; exec bash'" \
        --new-tab \
        -T "Aba 3: Máquina de Estados" -e "bash -c '${source_cmd}; sleep 8; ros2 run ${PACKAGE_NAME} state_machine_node; exec bash'"
    # ======================== FIM DA CORREÇÃO =========================
}

# PARA todos os processos relacionados à aplicação.
func_stop() {
    echo -e "${RED}=== Parando todos os processos da aplicação ===${NC}"
    pkill -f terminator || true
    pkill -f "gz sim" || true
    pkill -f "ros2 launch ${PACKAGE_NAME}" || true
    pkill -f "ros2 run ${PACKAGE_NAME}" || true
    echo -e "${GREEN}Processos finalizados.${NC}"
}

# LIMPA o workspace, removendo os diretórios de compilação.
func_nuke() {
    echo -e "${RED}🔥 ATENÇÃO! Isso irá parar todos os processos e apagar os diretórios 'build', 'install' e 'log'.${NC}"
    read -p "Você tem certeza? (s/n): " confirm
    if [[ "$confirm" != "s" ]]; then
        echo "Operação cancelada."
        exit 0
    fi

    func_stop
    echo -e "${YELLOW}Limpando diretórios do workspace...${NC}"
    cd "${WORKSPACE}"
    rm -rf build install log
    echo -e "${GREEN}Limpeza concluída.${NC}"
}


# MOSTRA o menu de ajuda.
func_usage() {
    echo -e "${YELLOW}Painel de Controle da Aplicação ROS${NC}"
    echo -e "------------------------------------"
    echo -e "Uso: runapp [comando]\n"
    echo -e "Comandos de Gerenciamento:"
    echo -e "  ${GREEN}build${NC}        - Compila o workspace."
    echo -e "  ${GREEN}update${NC}       - Instala/atualiza as dependências (rosdep)."
    echo -e "\nComandos de Execução:"
    echo -e "  ${BLUE}start${NC}        - Inicia a aplicação completa (Simulação, Planner, Controle)."
    echo -e "  ${RED}stop${NC}         - Para todos os processos da aplicação."
    echo -e "\nComandos Destrutivos:"
    echo -e "  ${RED}nuke${NC}         - Para tudo e limpa os diretórios 'build', 'install', 'log'."
}

# --- Roteador Principal de Comandos ---
if [ $# -eq 0 ]; then
    func_usage
    exit 0
fi

case "$1" in
    start)
        func_start
        ;;
    stop)
        func_stop
        ;;
    build)
        func_build
        ;;
    update)
        func_update_deps
        ;;
    nuke)
        func_nuke
        ;;
    *)
        echo -e "${RED}Erro: Comando '$1' desconhecido.${NC}"
        func_usage
        exit 1
        ;;
esac