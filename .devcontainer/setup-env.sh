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
# Isso é crucial porque os arquivos montados do Windows podem pertencer ao 'root'
# dentro do contêiner, o que causaria erros de permissão na compilação.
fix_permissions() {
    log_info "Verificando e ajustando permissões do workspace..."
    # '-R' (recursivo) aplica a mudança a todos os arquivos e pastas.
    sudo chown -R ros:ros /home/ros/ros2_ws
}

# Configura as variáveis de ambiente para permitir que aplicações com interface gráfica (GUI)
# como o Gazebo e o RViz, que rodam no contêiner, sejam exibidas no seu monitor do Windows.
setup_gui_environment() {
    # Detecta o IP do seu computador (host) e o define como o 'DISPLAY',
    # dizendo às aplicações gráficas para onde enviar suas janelas.
    export DISPLAY=$(grep -m 1 nameserver /etc/resolv.conf | awk '{print $2}'):0
    
    # Verifica se uma GPU NVIDIA está acessível dentro do contêiner.
    if command -v nvidia-smi &>/dev/null; then
        log_info "GPU NVIDIA detectada. Usando aceleração por hardware."
        # Imprime o nome da GPU para confirmação.
        nvidia-smi --query-gpu=name --format=csv,noheader
    else
        log_warn "GPU NVIDIA não detectada. Usando renderização por software (pode ser mais lento)."
        # Força o uso da renderização por software (CPU) se nenhuma GPU for encontrada.
        # Isso garante que a GUI funcione em qualquer máquina.
        export LIBGL_ALWAYS_SOFTWARE=1
        export GALLIUM_DRIVER=llvmpipe
    fi
}

# Prepara o workspace do ROS, instalando dependências e compilando o código.
setup_workspace() {
    # Navega para a raiz do workspace para executar os comandos a partir do local correto.
    cd /home/ros/ros2_ws

    log_info "Instalando dependências do projeto com rosdep..."
    # 'rosdep install' lê todos os arquivos 'package.xml' na pasta 'src',
    # encontra as dependências listadas (ex: <depend>ros2_control</depend>)
    # e as instala usando 'apt-get'.
    # '--from-paths src': Procura pacotes na pasta 'src'.
    # '--ignore-src': Não tenta instalar pacotes que já estão no código-fonte.
    # '-r': Continua mesmo que um pacote falhe.
    # '-y': Responde 'sim' para todas as perguntas.
    rosdep install --from-paths src --ignore-src -r -y
    if [ $? -ne 0 ]; then
        log_error "Falha ao instalar dependências com rosdep. Verifique seu package.xml."
        exit 1
    fi

    log_info "Construindo o workspace com colcon..."
    # Compila todo o código-fonte na pasta 'src'.
    # O comando completo é usado aqui em vez do alias 'build' porque aliases
    # do .bashrc não estão disponíveis em scripts não-interativos como este.
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    if [ $? -ne 0 ]; then
        log_error "Falha na compilação do workspace. Verifique os erros de compilação acima."
        exit 1
    fi
}

# --- Execução Principal do Script ---
log_info "Iniciando configuração do ambiente de desenvolvimento ROS 2..."

# Chama as funções na ordem correta.
fix_permissions
setup_gui_environment

# Ativa o ambiente base do ROS 2 Humble. Isso é necessário para que
# comandos como 'rosdep' e 'colcon' sejam encontrados.
source /opt/ros/humble/setup.bash

setup_workspace

log_info "Ambiente configurado com sucesso! O contêiner está pronto para uso."

# ÚNICA ALTERAÇÃO: A seção abaixo foi removida.
# Motivo: A modificação de arquivos de configuração como o .bashrc deve ser feita
# uma única vez no Dockerfile para evitar duplicação e lentidão.
# As mensagens informativas também foram removidas para manter o log do postCreateCommand limpo.
# source ~/.bashrc  
