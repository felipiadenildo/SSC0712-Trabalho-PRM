# Use o ROS Humble como base
ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

# Evita prompts interativos durante a construção da imagem
ENV DEBIAN_FRONTEND=noninteractive

# Instala todas as dependências do sistema e do ROS de uma vez
# Isso otimiza as camadas do Docker
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    vim \
    terminator \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-topic-tools \
    dbus-x11 \
    libglib2.0-0 \
    # Limpa o cache do apt para reduzir o tamanho da imagem
    && rm -rf /var/lib/apt/lists/*

# Inicializa o rosdep para que esteja pronto para uso
RUN rosdep init && rosdep update

# Cria um usuário 'ros' para não rodar como root
# Adiciona o usuário aos grupos 'sudo' (para permissões de admin) e 'video' (para acesso a hardware gráfico)
RUN useradd -m -u 1000 -G sudo,video ros && \
    # Permite que o usuário 'ros' execute sudo sem senha
    echo "ros ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ros-nopasswd

# Muda para o usuário 'ros' para os próximos comandos
USER ros
# Define o diretório de trabalho padrão para o usuário 'ros'
WORKDIR /home/ros

# Cria a estrutura do workspace
RUN mkdir -p /home/ros/ros2_ws/src

# Configura o ambiente no .bashrc
# Isso garante que todo novo terminal aberto terá o ambiente ROS configurado
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    # NOTA: A linha para o workspace local foi movida para ser adicionada após a compilação
    echo "alias build='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo'" >> ~/.bashrc && \
    echo "alias s_ws='source /home/ros/ros2_ws/install/setup.bash'" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/home/ros/ros2_ws/src/prm/models:\${GAZEBO_MODEL_PATH}" >> ~/.bashrc && \
    echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc

# Copia os scripts de execução para dentro do contêiner
# O --chown garante que o usuário 'ros' seja o dono dos arquivos
COPY --chown=ros:ros .devcontainer/setup-env.sh .devcontainer/run-app.sh /home/ros/scripts/
# Torna os scripts executáveis
RUN sudo chmod +x /home/ros/scripts/*.sh

# Adiciona o diretório de scripts ao PATH do sistema
ENV PATH="/home/ros/scripts:${PATH}"

# Define o diretório de trabalho final para o workspace do ROS
WORKDIR /home/ros/ros2_ws

# Comando para manter o contêiner em execução em segundo plano
CMD ["tail", "-f", "/dev/null"]
