#!/bin/bash

# Script simplificado para controlar a compilação e execução do projeto ROS 2
# Use: ./run.sh [opções]   (ex: ./run.sh irt)
#      ./run.sh stop         (para encerrar tudo)

# --- Configurações ---
ROS2_WORKSPACE="/ros2_ws"
GAZEBO_WORLD="empty_arena.sdf"
LAUNCH_PACKAGE="prm"

# --- Funções ---

# Encerra todos os processos e terminais da simulação
cleanup() {
    echo "--- Encerrando simulação e todos os processos relacionados... ---"
    # Mata os terminais pelo título único. O '-f' busca por toda a linha de comando.
    pkill -f "ROS2 - SIMULAÇÃO Gazebo" && echo "✅ Terminal da Simulação encerrado."
    pkill -f "ROS2 - ROBÔ e Controle" && echo "✅ Terminal do Robô encerrado."
    pkill -f "ROS2 - Teste" && echo "✅ Terminal de Teste encerrado."
    sleep 1
    # Garante que os processos do Gazebo sejam finalizados
    killall -q gzserver gzclient rviz2
    echo "Limpeza concluída. 👋"
}

# Exibe a ajuda do script
show_help() {
    echo "Uso: ./run.sh [opções]"
    echo "Opções de inicialização (podem ser combinadas):"
    echo "  i     -> Inicia o terminal da Simulação com Gazebo."
    echo "  r     -> Inicia o terminal do Robô (controladores e RViz)."
    echo "  t     -> Inicia um terminal de Teste para comandos manuais."
    echo "  Exemplos: ./run.sh i    (só simulação)"
    echo "            ./run.sh ir   (simulação e robô)"
    echo "            ./run.sh irt  (tudo)"
    echo ""
    echo "Opção de encerramento:"
    echo "  stop  -> Encerra todos os terminais e processos da simulação."
}

# --- Lógica Principal ---

# Se o comando for "stop", limpa tudo e sai.
if [ "$1" == "stop" ]; then
    cleanup
    exit 0
fi

# Se nenhum argumento for passado, mostra a ajuda e sai.
if [ -z "$1" ]; then
    show_help
    exit 1
fi

# --- Preparação e Compilação (executado para qualquer opção de inicialização) ---
echo "--- Preparando o ambiente e compilando o workspace... ---"
cd "$ROS2_WORKSPACE" || exit 1
colcon build || exit 1
source install/setup.bash || exit 1
echo "Ambiente pronto. ✅"

# --- Lançamento dos Terminais com base nos parâmetros ---
echo "--- Iniciando terminais solicitados ($1)... ---"

# Verifica se 'i' está no argumento para iniciar a SIMULAÇÃO
if [[ "$1" == *i* ]]; then
    # O '2>/dev/null' OCULTA a mensagem "Couldn't connect to accessibility bus"
    gnome-terminal --title="ROS2 - SIMULAÇÃO Gazebo" -- bash -c "source install/setup.bash; ros2 launch ${LAUNCH_PACKAGE} inicia_simulacao.launch.py world:=${GAZEBO_WORLD}; exec bash" &
    echo "🚀 Terminal da Simulação iniciado."
fi

# Verifica se 'r' está no argumento para iniciar o ROBÔ
if [[ "$1" == *r* ]]; then
    gnome-terminal --title="ROS2 - ROBÔ e Controle" -- bash -c "source install/setup.bash; ros2 launch ${LAUNCH_PACKAGE} carrega_robo.launch.py; exec bash" &
    echo "🤖 Terminal do Robô iniciado."
fi

# Verifica se 't' está no argumento para iniciar o terminal de TESTE
if [[ "$1" == *t* ]]; then
    gnome-terminal --title="ROS2 - Teste" -- bash -c "source install/setup.bash; echo 'Use este terminal para comandos manuais.'; exec bash" &
    echo "🧪 Terminal de Teste iniciado."
fi

echo -e "\nProcesso de inicialização concluído! ✨"