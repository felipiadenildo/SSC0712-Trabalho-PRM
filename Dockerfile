# Use a imagem oficial do ROS 2 Humble como base. A versão 'desktop' é uma ótima escolha
# pois já inclui RViz, rclpy, e muitas outras dependências comuns.
FROM osrf/ros:humble-desktop

# Define o shell padrão para comandos futuros
SHELL ["/bin/bash", "-c"]

# Define o frontend do apt-get como não interativo para evitar que ele trave em prompts
ENV DEBIAN_FRONTEND=noninteractive

# --- PASSO 1: Instalação de Dependências Adicionais ---
# Atualiza os pacotes e instala apenas o que NÃO vem na imagem 'desktop' mas é
# necessário para o projeto ou para o ambiente de desenvolvimento.
RUN apt-get update && apt-get install -y \
    # Ferramentas essenciais de desenvolvimento
    git \
    nano \
    python3-pip \
    gnome-terminal \
    psmisc \
    dbus-x11 \
    # Dependências específicas do projeto (baseado no package.xml e launch files)
    ros-humble-ros-gz \
    ros-humble-gazebo-ros2-control \
    ros-humble-teleop-twist-keyboard \
    ros-humble-topic-tools \
    && rm -rf /var/lib/apt/lists/*

# --- PASSO 2: Criação e Configuração do Workspace ---
# Cria o diretório do workspace
WORKDIR /ros2_ws

# In your Dockerfile, before starting services or copying entrypoint
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# Copia os arquivos do projeto para a pasta src do workspace
# Nota: Esta linha é para quando se constrói a imagem com o código-fonte dentro.
# Se estiver usando o devcontainer.json com 'mounts', o código local irá sobrepor este.
COPY . /ros2_ws/src/prm

# --- PASSO 3: Instalação de Dependências do ROS com rosdep ---
# Esta é a forma mais robusta de garantir que todas as dependências do package.xml sejam instaladas.
RUN source /opt/ros/humble/setup.bash && \
    apt-get update && \
    rosdep init || echo "rosdep já inicializado." && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# --- PASSO 4: Build do Workspace ---
# Compila o workspace. O 'source' garante que o ambiente ROS esteja ativo para o colcon.
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# --- PASSO 5: Configuração Permanente do Ambiente ---
# Adiciona os comandos de source ao .bashrc para que qualquer terminal interativo
# novo já venha com o ambiente ROS configurado.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# (Opcional, mas recomendado) Define um ponto de entrada que configura o ambiente
# para qualquer comando executado no contêiner.
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# O comando padrão a ser executado se nenhum for fornecido ao 'docker run'.
CMD ["bash"]

