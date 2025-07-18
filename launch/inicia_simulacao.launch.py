import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Caminhos e Pacotes ---
    # Encontra o diretório de instalação do pacote 'prm' de forma robusta.
    pkg_prm = get_package_share_directory('prm')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # --- Declaração de Argumentos do Launch ---
    # Permite escolher o mundo a ser carregado via linha de comando.
    # Ex: ros2 launch prm inicia_simulacao.launch.py world:=arena_paredes.sdf
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_prm, 'world', 'empty_arena.sdf'),
        description='Caminho completo para o arquivo de mundo SDF a ser carregado.'
    )

    # --- Inicialização do Simulador Gazebo (Forma Padrão) ---
    # Inclui o launch file padrão do Gazebo, que é mais flexível.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            # Permite controlar a GUI. Mude para 'true' para modo headless.
            'headless': 'false', 
            # Passa o caminho do mundo selecionado para o Gazebo.
            'world': LaunchConfiguration('world')
        }.items()
    )
    
    # ===================== INÍCIO DA CORREÇÃO CRÍTICA =====================

    # 1. CARREGA A DESCRIÇÃO DO ROBÔ E OS CONTROLADORES
    # Inclui o launch file que lida com o URDF, publicadores de estado e ros2_control.
    carrega_robo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_prm, 'launch', 'carrega_robo.launch.py')
        )
    )

    # 2. INVOCA O ROBÔ NO MUNDO
    # Executa o nó 'spawn_entity' do Gazebo para adicionar o robô na simulação.
    # Ele lê a descrição do robô do tópico '/robot_description'.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'prm_robot'],
        output='screen'
    )
    
    # ====================== FIM DA CORREÇÃO CRÍTICA =======================
    
    # --- Ponte Gazebo <-> ROS 2 (Bridge) ---
    # Necessária para que o ROS 2 receba dados de sensores e do relógio do Gazebo.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # Necessário para controladores como o diff_drive_controller
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"
            # Adicione outros tópicos aqui se precisar (ex: /odom, /scan)
        ],
        output="screen",
    )

    # --- Descrição Completa do Lançamento ---
    # Retorna a lista de todas as ações que devem ser executadas.
    # A ordem importa: o Gazebo deve iniciar antes de tentarmos adicionar o robô.
    return LaunchDescription([
        world_arg,
        gazebo,
        carrega_robo, # Carrega a descrição do robô na memória
        spawn_entity, # Adiciona o robô na simulação
        bridge        # Inicia a ponte de comunicação
    ])