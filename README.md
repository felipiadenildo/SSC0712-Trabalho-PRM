# PRM - Programação de Robôs Móveis

**Disciplina SSC0712** Oferecida para os cursos de Engenharia de Computação e áreas afins na **USP São Carlos**

Este repositório contém o material da disciplina *Programação de Robôs Móveis*, focada no desenvolvimento de soluções em robótica móvel utilizando **ROS 2 Humble** e o simulador **Gazebo Fortress**.

Felipi Adenildo Soares Sousa 10438790
Gustavo Wadas Lopes 12745640

## Como Funciona

Este projeto implementa um sistema autônomo de "captura da bandeira" utilizando uma arquitetura modular baseada em nós do ROS 2. Cada nó tem uma responsabilidade específica, comunicando-se através de tópicos para executar a missão.

  * **Nó de Visão (`vision_node`)**: Atua como os "olhos" do robô. Ele processa o feed da câmera para detectar a **bandeira azul** e a **base amarela** usando máscaras de cor e publica a localização desses objetos (em coordenadas de pixel) em tópicos dedicados.

  * **Nó Planejador de Caminho (`path_planner_node`)**: É o "navegador". Ele escuta por um alvo (coordenadas no mundo) e, utilizando o mapa de ocupação gerado pelo robô, calcula a rota mais segura e eficiente usando um algoritmo customizado. O caminho resultante é publicado como uma sequência de pontos.

  * **Nó da Máquina de Estados (`state_machine_node`)**: É o "cérebro" da operação. Ele orquestra a missão inteira, decidindo o que fazer com base no estado atual e nos dados dos outros nós. Ele comanda o robô para procurar a bandeira, solicita um caminho ao planejador, segue o caminho, controla a garra para captura e, finalmente, retorna à base para depositar a bandeira.

## Como Executar a Missão

Para executar a missão completa do robô, siga os passos abaixo. É necessário que o ambiente ROS 2 já esteja configurado e que todas as dependências do projeto tenham sido instaladas. Os comandos devem ser executados a partir da raiz do seu workspace (ex: `~/ros2_ws`).

### Passo 1: Compilar o Workspace

Antes de executar, você precisa compilar todos os pacotes do projeto. Este comando cria os executáveis e os arquivos de configuração necessários na pasta `install`.

```bash
colcon build
```

### Passo 2: Habilitar o Ambiente

Após compilar, você deve "habilitar" o ambiente para que o ROS 2 saiba onde encontrar seus pacotes e executáveis. **Este passo é crucial e deve ser repetido para cada novo terminal que você abrir.**

```bash
source install/setup.bash
```

### Passo 3: Iniciar os Nós em Ordem

A execução requer **5 terminais separados**. Abra cada um, navegue até a raiz do workspace (`cd ~/ros2_ws`) e execute o comando `source install/setup.bash` em cada um antes de rodar os comandos abaixo.

1.  **Terminal 1 - Iniciar o Mundo (Gazebo):**
    Este comando carrega o cenário da simulação, mas ainda sem o robô.

    ```bash
    ros2 launch prm inicia_simulacao.launch.py world:=arena_cilindros.sdf
    ```

2.  **Terminal 2 - Carregar o Robô:**
    Este comando adiciona o robô ao mundo, carrega seus controladores (rodas e garra) e inicia o RViz para visualização.

    ```bash
    ros2 launch prm carrega_robo.py
    ```

3.  **Terminal 3 - Iniciar o Planejador de Caminho:**
    Este nó ficará aguardando por solicitações de caminho.

    ```bash
    ros2 run prm path_planner_node
    ```

4.  **Terminal 4 - Iniciar o Nó de Visão:**
    Este nó começará a processar as imagens da câmera do robô.

    ```bash
    ros2 run prm vision_node
    ```

5.  **Terminal 5 - Iniciar a Máquina de Estados (Início da Missão):**
    Este é o último comando. Ele inicia a lógica central que fará o robô se mover e executar a missão de captura da bandeira.

    ```bash
    ros2 run prm state_machine_node
    ```

Agora, o robô deve começar a procurar pela bandeira e executar a missão de forma autônoma.

-----

### Adendo: Otimização para Desempenho (Modo Headless)

Para computadores com recursos limitados, rodar a interface gráfica do Gazebo pode consumir muita performance. É possível iniciar apenas o servidor de simulação (a parte que calcula a física), sem a parte visual, economizando recursos.

Para isso, adicione a flag `-s` (server-only) ao comando de inicialização do Gazebo.

**Ação:**

1.  Abra o arquivo `launch/inicia_simulacao.launch.py`.

2.  Localize a seção `ExecuteProcess` que inicia o Gazebo.

3.  Adicione `'-s',` à lista `cmd`, como mostrado abaixo:

    ```python
    # Dentro de inicia_simulacao.launch.py

    gazebo = ExecuteProcess(
        # Adicione o '-s' para rodar sem a GUI (headless)
        cmd=['ruby', FindExecutable(name="ign"), 'gazebo', '-r', '-s', '-v', gz_verbosity, world_path],
        output='screen',
        additional_env=gz_env,
        shell=False,
    )
    ```

Ao lançar a simulação com esta alteração, a janela do Gazebo não abrirá, mas você ainda poderá ver o robô e o mapa através do **RViz**, que é muito mais leve.