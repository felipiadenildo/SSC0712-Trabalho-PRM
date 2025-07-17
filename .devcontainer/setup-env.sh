#!/bin/bash
# Este script configura o ambiente de desenvolvimento ROS 2 dentro do contêiner.
# Ele deve ser executado uma vez por sessão de terminal para garantir que tudo esteja pronto.
set -eo pipefail

# --- Definição de Cores para o Output ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# --- Funções de Log ---
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# --- Funções Principais ---

# Garante que o usuário 'ros' seja o dono dos arquivos do workspace.
# Isso previne problemas de permissão que podem ocorrer ao montar volumes do host.
fix_permissions() {
    log_info "Verificando e ajustando permissões do workspace..."
    sudo chown -R ros:ros /home/ros/ros2_ws
}

# Configura as variáveis de ambiente para permitir que aplicações gráficas (GUI)
# como o Gazebo e o RViz funcionem corretamente.
setup_gui_environment() {
    # Detecta o IP do host para encaminhamento X11 (essencial para GUIs)
    export DISPLAY=$(grep -m 1 nameserver /etc/resolv.conf | awk '{print $2}'):0
    
    # Verifica se uma GPU NVIDIA está disponível no host
    if command -v nvidia-smi &>/dev/null; then
        log_info "GPU NVIDIA detectada. Usando aceleração de hardware."
        nvidia-smi --query-gpu=name --format=csv,noheader
    else
        log_warn "GPU NVIDIA não detectada. Usando renderização por software (pode ser mais lento)."
        export LIBGL_ALWAYS_SOFTWARE=1
        export GALLIUM_DRIVER=llvmpipe
    fi
}

# Prepara o workspace do ROS, instalando dependências e compilando se necessário.
setup_workspace() {
    # Navega para a raiz do workspace
    cd /home/ros/ros2_ws

    log_info "Instalando dependências do projeto com rosdep..."
    # Instala todas as dependências listadas no package.xml de todos os pacotes na pasta 'src'
    # Esta é a correção para o erro "package not found" que você teve.
    rosdep install --from-paths src --ignore-src -r -y
    if [ $? -ne 0 ]; then
        log_error "Falha ao instalar dependências com rosdep. Verifique seu package.xml e a conexão com a internet."
        exit 1
    fi

    log_info "Construindo o workspace com colcon..."
    # Compila o projeto. O alias 'build' foi definido no Dockerfile.
    build
    if [ $? -ne 0 ]; then
        log_error "Falha na compilação do workspace. Verifique os erros acima."
        exit 1
    fi
    
    # Ativa o ambiente do workspace recém-compilado
    log_info "Ativando o ambiente do workspace local."
    source /home/ros/ros2_ws/install/setup.bash
}

# --- Execução Principal do Script ---
log_info "Iniciando configuração do ambiente de desenvolvimento ROS 2..."

fix_permissions
setup_gui_environment

# Ativa o ambiente base do ROS 2 Humble
source /opt/ros/humble/setup.bash

setup_workspace

log_info "Ambiente configurado com sucesso!"
echo -e "Use o alias ${YELLOW}rosapp${NC} para executar a aplicação (ex: rosapp sim)"

# Executa qualquer comando passado como argumento para este script
# Isso permite encadear comandos, como: ./setup-env.sh rosapp sim
exec "$@"
