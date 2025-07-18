# Define um argumento para a versão do ROS, permitindo flexibilidade.
# O valor padrão é 'humble'.
ARG ROS_DISTRO=humble
# Inicia a construção a partir da imagem oficial do ROS 2 Humble com ambiente de desktop completo.
# Esta imagem já contém o ROS, Gazebo, RViz e muitas outras ferramentas.
FROM osrf/ros:${ROS_DISTRO}-desktop

# Define uma variável de ambiente para evitar que pacotes como o 'tzdata'
# peçam inputs interativos durante a instalação.
ENV DEBIAN_FRONTEND=noninteractive

# Bloco de instalação de dependências.
# 'apt-get update' atualiza a lista de pacotes disponíveis.
RUN apt-get update && apt-get install -y \
    # Ferramentas de desenvolvimento Python
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    # Compiladores essenciais (g++, make, etc.)
    build-essential \
    # Editor de texto de terminal
    vim \
    # Emulador de terminal avançado que permite dividir a tela
    terminator \
    # Utilitários gráficos para OpenGL
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    # Pontes de comunicação entre ROS 2 e Gazebo (Ignition)
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    # Ferramenta para processar arquivos de robô .xacro
    ros-${ROS_DISTRO}-xacro \
    # Ferramentas para manipulação de tópicos ROS
    ros-${ROS_DISTRO}-topic-tools \
    # Instala explicitamente os pacotes do ROS 2 Control.
    # Esta é a forma mais garantida de tê-los no ambiente.
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    # Dependências para aplicações gráficas se comunicarem via D-Bus
    dbus-x11 \
    libglib2.0-0 \
    # Ao final, limpa o cache do apt para reduzir o tamanho final da imagem Docker.
    && rm -rf /var/lib/apt/lists/*

# Inicializa o rosdep a nível de sistema (como root).
# O '|| true' é um truque para ignorar o erro caso o rosdep já tenha sido
# inicializado na imagem base, evitando que a construção falhe.
RUN rosdep init || true

# Cria um usuário não-root chamado 'ros' para maior segurança.
# -m: cria o diretório home (/home/ros)
# -u 1000: define o ID do usuário como 1000 (padrão para o primeiro usuário)
# -G sudo,video: adiciona o usuário aos grupos 'sudo' (para permissões de admin)
#                e 'video' (para acesso a hardware gráfico).
RUN useradd -m -u 1000 -G sudo,video ros && \
    # Concede permissões de 'sudo' ao usuário 'ros' sem precisar de senha.
    # Isso é conveniente em um ambiente de desenvolvimento.
    echo "ros ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ros-nopasswd

# Muda o contexto dos próximos comandos para o usuário 'ros'.
USER ros
# Define o diretório de trabalho padrão para /home/ros.
WORKDIR /home/ros

# Atualiza o cache do rosdep. Este comando é executado como usuário 'ros',
# o que garante que o cache seja criado no diretório correto (/home/ros/.ros/).
# Esta é uma correção crucial que resolve muitos dos erros anteriores.
RUN rosdep update

# Cria a estrutura de pastas para o workspace do ROS 2.
RUN mkdir -p /home/ros/ros2_ws/src

# --- Bloco de configuração do .bashrc ---
# Adiciona comandos ao ~/.bashrc do usuário 'ros'.
# Estes comandos serão executados toda vez que um novo terminal bash for aberto.
RUN echo "" >> ~/.bashrc && \
    echo "# --- Configurações do Ambiente ROS Customizado ---" >> ~/.bashrc && \
    # Ativa o ambiente base do ROS 2 Humble.
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    # Ativa o ambiente do workspace local, mas SOMENTE se ele já tiver sido compilado.
    # Isso evita erros na primeira vez que o contêiner é iniciado.
    echo "if [ -f /home/ros/ros2_ws/install/setup.bash ]; then source /home/ros/ros2_ws/install/setup.bash; fi" >> ~/.bashrc && \
    # Define o caminho para os modelos do Gazebo.
    echo "export GAZEBO_MODEL_PATH=/home/ros/ros2_ws/src/prm/models:\${GAZEBO_MODEL_PATH}" >> ~/.bashrc && \
    # Configuração para melhorar a compatibilidade de GUIs em ambientes remotos.
    echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc && \
    # --- Aliases (Atalhos) ---
    echo "# Aliases para facilitar o desenvolvimento" >> ~/.bashrc && \
    # ÚNICA ALTERAÇÃO: Agrupando todos os aliases para melhor organização.
    echo "alias build='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo'" >> ~/.bashrc && \
    echo "alias runapp='/home/ros/scripts/run-app.sh'" >> ~/.bashrc && \
    # Copia a configuração para o .profile para garantir que seja carregada em shells de login.
    echo cp ~/.bashrc ~/.profile

# Copia os scripts da pasta .devcontainer do host para a pasta /home/ros/scripts/ dentro do contêiner.
# O '--chown' garante que o usuário 'ros' seja o dono dos arquivos copiados.
COPY --chown=ros:ros .devcontainer/setup-env.sh .devcontainer/run-app.sh /home/ros/scripts/
# Dá permissão de execução para todos os scripts na pasta.
RUN sudo chmod +x /home/ros/scripts/*.sh

# Adiciona a pasta de scripts ao PATH do sistema.
# Isso permite que os scripts (como 'run-app.sh') sejam chamados de qualquer diretório.
ENV PATH="/home/ros/scripts:${PATH}"

# Define o diretório de trabalho final para a raiz do workspace.
WORKDIR /home/ros/ros2_ws

# Comando padrão que o contêiner executa ao iniciar.
# 'tail -f /dev/null' é um truque para manter o contêiner rodando indefinidamente em segundo plano.
CMD ["tail", "-f", "/dev/null"]