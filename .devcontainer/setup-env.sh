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
    log_info "Verificando e ajustando permissões do workspace..."
    # '-R' (recursivo) aplica a mudança a todos os arquivos e pastas.
    sudo chown -R ros:ros /home/ros/ros2_ws
    # Corrige permissões do workspace
    sudo chown -R ros:ros /home/ros/ros2_ws
    sudo chmod -R u+rw /home/ros/ros2_ws
}

# Configura as variáveis de ambiente para permitir que aplicações com GUI
# (Gazebo, RViz), que rodam no contêiner, sejam exibidas no seu monitor local.
setup_gui_environment() {
    log_info "Configurando ambiente para exibição de GUI..."
    # Detecta o IP do seu computador (host) e o define como o 'DISPLAY',
    # dizendo às aplicações gráficas para onde enviar suas janelas.
    export DISPLAY=$(grep -m 1 nameserver /etc/resolv.conf | awk '{print $2}'):0
    
    # Verifica se uma GPU NVIDIA está acessível dentro do contêiner.
    if command -v nvidia-smi &>/dev/null; then
        log_info "GPU NVIDIA detectada. Usando aceleração por hardware."
        nvidia-smi --query-gpu=name --format=csv,noheader
    else
        log_warn "GPU NVIDIA não detectada. Usando renderização por software (pode ser mais lento)."
        # Força o uso da renderização por software (CPU) se nenhuma GPU for encontrada.
        export LIBGL_ALWAYS_SOFTWARE=1
    fi
}

# ======================= INÍCIO DA CORREÇÃO PRINCIPAL =======================
# Prepara a estrutura do workspace para que seja compatível com o colcon.
# Esta é a etapa que resolve o problema do 'src' que discutimos.
prepare_workspace_structure() {
    log_info "Preparando a estrutura do workspace ROS 2..."
    cd /home/ros/ros2_ws

    # [cite_start]O Dockerfile já cria a pasta '/home/ros/ros2_ws/src'[cite: 126], mas garantimos aqui.
    mkdir -p src

    # Cria um link simbólico do seu pacote 'prm' (que foi montado na raiz do workspace)
    # para dentro da pasta 'src', que é onde o 'colcon' espera encontrá-lo.
    # O 'if' evita erros se o script for executado novamente.
    if [ ! -L "src/prm" ]; then
        ln -s /home/ros/ros2_ws/prm src/prm
        log_info "Link simbólico para o pacote 'prm' criado em 'src/'."
    else
        log_warn "Link simbólico para 'prm' já existe. Nenhuma ação necessária."
    fi
}
# ======================== FIM DA CORREÇÃO PRINCIPAL =========================

# Instala dependências e compila o código-fonte do workspace.
setup_workspace() {
    cd /home/ros/ros2_ws

    log_info "Instalando dependências do projeto com rosdep..."
    # 'rosdep install' lê todos os 'package.xml' na pasta 'src', encontra as
    # dependências e as instala com 'apt-get'.
    rosdep install --from-paths src --ignore-src -r -y
    if [ $? -ne 0 ]; then
        log_error "Falha ao instalar dependências com rosdep. Verifique seu package.xml."
        exit 1
    fi

    log_info "Construindo o workspace com colcon..."
    # Compila todo o código-fonte na pasta 'src'.
    # O comando completo é usado aqui porque aliases do .bashrc não estão
    # disponíveis em scripts não-interativos como este.
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
setup_gui_environment

# Ativa o ambiente base do ROS 2 Humble. Isso é necessário para que
# [cite_start]comandos como 'rosdep' e 'colcon' sejam encontrados no PATH. [cite: 129]
source /opt/ros/humble/setup.bash
source install/setup.bash

# Prepara a estrutura de pastas ANTES de tentar instalar e compilar.
prepare_workspace_structure
setup_workspace

log_info "Ambiente configurado com sucesso! O contêiner está pronto para uso."