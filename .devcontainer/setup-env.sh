#!/bin/bash
# Este script automatiza a configuração inicial do ambiente de desenvolvimento ROS 2
# dentro do contêiner. É executado automaticamente uma vez pelo VS Code
# (via postCreateCommand) após a criação do contêiner.
# 'set -e' garante que o script pare imediatamente se algum comando falhar.
set -e

# --- Definição de Cores para o Output ---
# Define variáveis para cores, facilitando a leitura dos logs.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color (reseta a cor)

# --- Funções de Log ---
# Funções auxiliares para imprimir mensagens formatadas no terminal.
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

# Garante que o usuário 'ros' seja o dono de todos os arquivos do workspace.
# Crucial porque arquivos montados do host (Windows/macOS) podem pertencer ao 'root'
# dentro do contêiner, o que causaria erros de permissão na compilação.
fix_permissions() {
    log_info "Verificando e ajustando permissões do workspace em /home/ros/ros2_ws..."
    # '-R' (recursivo) aplica a mudança a todos os arquivos e pastas.
    # Garante que o usuário 'ros' seja o dono e tenha permissões de leitura/escrita.
    sudo chown -R ros:ros /home/ros/ros2_ws
    sudo chmod -R u+rw /home/ros/ros2_ws
}

# Verifica a disponibilidade da GPU para aceleração gráfica.
check_gpu_environment() {
    log_info "Verificando ambiente para GUI..."
    # A variável DISPLAY já é configurada pelo devcontainer.json, que é o método mais robusto.
    # Esta função agora apenas verifica a presença da GPU para fins de log.
    if command -v nvidia-smi &>/dev/null; then
        log_info "GPU NVIDIA detectada. Usando aceleração por hardware."
        nvidia-smi --query-gpu=name --format=csv,noheader
    else
        log_warn "GPU NVIDIA não detectada. O sistema usará renderização por software (pode ser mais lento)."
    fi
}

# Instala dependências e compila o código-fonte do workspace.
setup_workspace() {
    # Navega para a raiz do workspace.
    cd /home/ros/ros2_ws

    # ✅ EDIÇÃO CRÍTICA: Atualiza a lista de pacotes do sistema ANTES de usar rosdep.
    # Isso garante que o 'apt' encontre todos os pacotes do ROS.
    log_info "Atualizando lista de pacotes do sistema (apt-get update)..."
    sudo apt-get update

    log_info "Instalando dependências do projeto com rosdep..."
    # 'rosdep install' lê todos os 'package.xml' na pasta 'src', encontra as
    # dependências e as instala.
    rosdep install --from-paths src --ignore-src -r -y
    if [ $? -ne 0 ]; then
        log_error "Falha ao instalar dependências com rosdep. Verifique seu package.xml."
        exit 1
    fi

    log_info "Construindo o workspace com colcon..."
    # Compila todo o código-fonte na pasta 'src'.
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    if [ $? -ne 0 ]; then
        log_error "Falha na compilação do workspace. Verifique os erros de compilação acima."
        exit 1
    fi
}

# --- Execução Principal do Script ---
log_info "Iniciando configuração do ambiente de desenvolvimento ROS 2..."

# Chama as funções na ordem correta e lógica.
fix_permissions
check_gpu_environment

# ✅ EDIÇÃO: Ativa o ambiente base do ROS 2 Humble. Isso é necessário para que
# comandos como 'rosdep' e 'colcon' sejam encontrados no PATH.
# A linha 'source install/setup.bash' foi removida pois o diretório 'install'
# ainda não existe neste ponto.
source /opt/ros/humble/setup.bash

# ✅ EDIÇÃO: A função 'prepare_workspace_structure' foi removida.
# Com a correção no 'devcontainer.json', o seu pacote 'prm' já é montado
# diretamente em '/home/ros/ros2_ws/src/prm', tornando o link simbólico desnecessário.
# O script agora vai direto para a instalação e compilação.
setup_workspace

log_info "Ambiente configurado com sucesso! O contêiner está pronto para uso."