# Define um argumento para a versão do ROS, permitindo flexibilidade.
# O valor padrão é 'humble'.
ARG ROS_DISTRO=humble
# Inicia a construção a partir da imagem oficial do ROS 2 Humble com ambiente de desktop completo.
# Esta imagem já contém o ROS, Gazebo, RViz e muitas outras ferramentas.
FROM osrf/ros:${ROS_DISTRO}-desktop

# Define uma variável de ambiente para evitar que pacotes como o 'tzdata'
# peçam inputs interativos durante a instalação.
ENV DEBIAN_FRONTEND=noninteractive
# Desativa o uso da extensão de memória compartilhada (MIT-SHM) para o X11.
# Isso resolve erros comuns de 'BadAccess' ao rodar aplicações com GUI em certos sistemas.
ENV QT_X11_NO_MITSHM=1

# Bloco de instalação de dependências do sistema e do ROS.
# 'apt-get update' atualiza a lista de pacotes disponíveis.
RUN apt-get update && apt-get install -y \
    # Ferramentas de desenvolvimento Python e de compilação
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    # Editores e utilitários de conveniência
    vim \
    terminator \
    # Utilitários gráficos e bibliotecas para OpenGL
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    # Pontes de comunicação entre ROS 2 e Gazebo (Ignition)
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    # Ferramenta para processar arquivos de descrição de robô .xacro
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-topic-tools \
    # Pacotes do ROS 2 Control para controle de juntas e atuadores
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    # Dependências para aplicações gráficas se comunicarem via D-Bus
    dbus-x11 \
    libglib2.0-0 \
    # Ao final, limpa o cache do apt para reduzir o tamanho final da imagem Docker.
    && rm -rf /var/lib/apt/lists/*

# Inicializa o rosdep a nível de sistema (como root).
# O '|| true' é um mecanismo de segurança para ignorar o erro caso o rosdep
# já tenha sido inicializado na imagem base, evitando que a construção falhe.
RUN rosdep init || true

# Cria um usuário não-root chamado 'ros' para maior segurança e compatibilidade.
# -m: cria o diretório home (/home/ros).
# -G sudo,video: adiciona o usuário aos grupos 'sudo' (para permissões de admin)
#                e 'video' (essencial para acesso a hardware gráfico/GUI).
RUN useradd -m -u 1000 -G sudo,video ros && \
    # Concede permissões de 'sudo' ao usuário 'ros' sem precisar de senha.
    # Isso é extremamente conveniente em um ambiente de desenvolvimento contido.
    echo "ros ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ros-nopasswd

# Muda o contexto dos próximos comandos para o usuário 'ros'.
USER ros
# Define o diretório de trabalho padrão para o home do novo usuário.
WORKDIR /home/ros

# Atualiza o cache do rosdep. Este comando DEVE ser executado como usuário 'ros',
# garantindo que o cache seja criado no diretório correto (/home/ros/.ros/).
RUN rosdep update

# ✅ BOA PRÁTICA: Cria a estrutura do workspace dentro do diretório home do usuário.
# Isso respeita a hierarquia do sistema de arquivos e evita problemas de permissão.
RUN mkdir -p /home/ros/ros2_ws/src

# --- Bloco de configuração do .bashrc ---
# Adiciona comandos ao ~/.bashrc do usuário 'ros'.
# Estes comandos serão executados automaticamente toda vez que um novo terminal for aberto.
RUN echo "" >> ~/.bashrc && \
    echo "# --- Configurações do Ambiente ROS Customizado ---" >> ~/.bashrc && \
    # Carrega o ambiente base do ROS 2 Humble.
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    # Carrega o ambiente do workspace local, mas SOMENTE se ele já tiver sido compilado.
    # A verificação 'if [ -f ... ]' evita erros na primeira vez que o contêiner é iniciado.
    echo "if [ -f /home/ros/ros2_ws/install/setup.bash ]; then source /home/ros/ros2_ws/install/setup.bash; fi" >> ~/.bashrc && \
    # Adiciona a pasta de modelos do seu pacote ao caminho de busca do Gazebo.
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/home/ros/ros2_ws/src/prm/models" >> ~/.bashrc && \
    # Configuração para melhorar a compatibilidade de GUIs em ambientes remotos.
    echo "export LIBGL_ALWAYS_INDIRECT=1" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    # --- Aliases (Atalhos de Conveniência) ---
    echo "# Aliases para facilitar o desenvolvimento" >> ~/.bashrc && \
    # ✅ EDIÇÃO: Alias 'build' melhorado para garantir que a compilação ocorra no diretório correto.
    echo "alias build='cd /home/ros/ros2_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo'" >> ~/.bashrc && \
    # Alias para o seu script de controle.
    echo "alias runapp='run-app.sh'" >> ~/.bashrc && \
    # Alias para carregar o ambiente do workspace manualmente, se necessário.
    echo "alias s_ws='source /home/ros/ros2_ws/install/setup.bash'" >> ~/.bashrc

# ✅ EDIÇÃO: Corrige a cópia do .bashrc para o .profile.
# O comando 'RUN' executa o comando, em vez de 'echo' que apenas o escreveria no arquivo.
RUN cp ~/.bashrc ~/.profile

# ✅ BOA PRÁTICA: Copia os scripts para uma pasta dedicada e organizada.
# O '--chown' garante que o usuário 'ros' seja o dono dos arquivos copiados.
COPY --chown=ros:ros .devcontainer/setup-env.sh .devcontainer/run-app.sh /home/ros/scripts/
# Dá permissão de execução para todos os scripts na pasta.
RUN chmod +x /home/ros/scripts/*.sh

# ✅ BOA PRÁTICA: Adiciona a pasta de scripts ao PATH do sistema.
# Isso permite que os scripts (como 'run-app.sh') sejam chamados de qualquer diretório.
ENV PATH="/home/ros/scripts:${PATH}"

# Define o diretório de trabalho final para a raiz do workspace.
# Esta é a pasta que será aberta por padrão no terminal do VS Code.
WORKDIR /home/ros/ros2_ws

# Comando padrão que o contêiner executa ao iniciar.
# 'tail -f /dev/null' é um truque para manter o contêiner rodando indefinidamente,
# permitindo que você se conecte a ele com o VS Code.
CMD ["tail", "-f", "/dev/null"]