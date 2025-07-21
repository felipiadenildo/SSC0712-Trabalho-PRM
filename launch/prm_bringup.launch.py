# /launch/prm_bringup.launch.py

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ===================================================================================
    # === 1. DECLARAÇÃO DE ARGUMENTOS DE LANÇAMENTO (CONFIGURAÇÕES) ===
    # ===================================================================================
    
    # Argumento para especificar o ficheiro de mundo a ser usado
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='arena_cilindros.sdf', # O seu mundo padrão com a bandeira
        description='Nome do ficheiro .sdf do mundo a ser carregado na pasta /world'
    )

    # Argumento para decidir se o RViz deve ser iniciado ou não
    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Inicia o RViz se for "true"'
    )

    # ===================================================================================
    # === 2. DEFINIÇÃO DE CAMINHOS E FICHEIROS ===
    # ===================================================================================

    # Diretório de partilha do pacote 'prm'
    pkg_prm_share = get_package_share_directory('prm')
    
    # Caminho completo para o ficheiro de mundo
    world_path = PathJoinSubstitution([
        pkg_prm_share,
        "world",
        LaunchConfiguration('world')
    ])

    # Caminho para o ficheiro de descrição do robô (XACRO)
    xacro_file_path = PathJoinSubstitution([
        pkg_prm_share,
        'description',
        'robot.urdf.xacro' # Assumindo que este é o nome do seu ficheiro principal
    ])

    # Caminho para o ficheiro de configuração do RViz
    rviz_config_file_path = PathJoinSubstitution([
        pkg_prm_share,
        'rviz',
        'rviz_config.rviz'
    ])

    # ===================================================================================
    # === 3. INICIALIZAÇÃO DO AMBIENTE DE SIMULAÇÃO (GAZEBO) ===
    # ===================================================================================

    # Inicia o servidor e o cliente do Gazebo (GUI)
    # Usamos o ExecuteProcess para ter controlo sobre os argumentos, como no seu exemplo
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '3', world_path],
        output='screen',
    )

    # ===================================================================================
    # === 4. PREPARAÇÃO E LANÇAMENTO DOS NÓS DO ROBÔ ===
    # ===================================================================================

    # Processa o ficheiro XACRO para gerar o URDF que o ROS 2 entende
    robot_description_content = Command(['xacro ', xacro_file_path])

    # Nó Robot State Publisher: Publica as transformações (TFs) do robô
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Nó para instanciar (spawn) o robô no Gazebo
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_prm_robot',
        arguments=[
            '-name', 'prm_robot',
            '-topic', 'robot_description',
            '-z', '0.2',
            '-x', '-2.0', # Posição inicial do robô
            '-y', '1.0'
        ],
        output='screen'
    )

    # Ponte de comunicação entre ROS 2 e Gazebo
    # Este bridge é essencial para que os sensores e atuadores funcionem
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Clock (essencial para a simulação)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Sensor Laser (Lidar)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Câmara
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Odometria (Ground Truth)
            '/model/prm_robot/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
        ],
        output='screen'
    )

    # ===================================================================================
    # === 5. LANÇAMENTO DOS CONTROLADORES (ROS2 CONTROL) ===
    # ===================================================================================

    # Carrega os controladores de juntas e do Diff Drive após o robô ser instanciado
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_base_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
        output='screen'
    )

    # ===================================================================================
    # === 6. LANÇAMENTO DOS NÓS DE LÓGICA DO PROJETO ===
    # ===================================================================================

    # Nó de Mapeamento (se necessário)
    robo_mapper_node = Node(
        package='prm',
        executable='robo_mapper',
        name='robo_mapper'
    )

    # Nó de Odometria (Ground Truth)
    ground_truth_odometry_node = Node(
        package='prm',
        executable='ground_truth_odometry',
        name='ground_truth_odometry'
    )

    # Nó de Visão Computacional (NOVO)
    vision_node = Node(
        package='prm',
        executable='vision_node',
        name='vision_node',
        output='screen'
    )
    
    # Nó de Planeamento de Caminho (NOVO)
    path_planner_node = Node(
        package='prm',
        executable='path_planner_node', # Usa o nome do executável definido no setup.py
        name='path_planner_node',
        output='screen'
    )

    # Nó da Máquina de Estados (NOVO)
    state_machine_node = Node(
        package='prm',
        executable='state_machine_node', # Usa o nome do executável definido no setup.py
        name='state_machine_node',
        output='screen'
    )

    # ===================================================================================
    # === 7. LANÇAMENTO DO RVIZ (VISUALIZAÇÃO) ===
    # ===================================================================================
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')), # Só inicia se use_rviz for 'true'
        output='screen'
    )

    # ===================================================================================
    # === 8. MONTAGEM DA DESCRIÇÃO FINAL DO LANÇAMENTO ===
    # ===================================================================================

    return LaunchDescription([
        # Argumentos
        declare_world_arg,
        declare_use_rviz_arg,

        # Iniciar Simulação
        start_gazebo,

        # Nós do Robô
        robot_state_publisher_node,
        spawn_robot_node,
        ros_gz_bridge,
        
        # Nós de Lógica e Utilitários
        ground_truth_odometry_node,
        robo_mapper_node,
        vision_node,
        path_planner_node,
        state_machine_node,

        # RViz
        rviz_node,

        # --- Eventos para Carregar Controladores na Ordem Correta ---
        # Regista um "ouvinte" que espera o nó 'spawn_prm_robot' terminar.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot_node,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        # Espera o 'joint_state_broadcaster' carregar para iniciar o controlador das rodas.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_diff_drive_base_controller],
            )
        ),
        # Espera o controlador das rodas carregar para iniciar o da garra.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_diff_drive_base_controller,
                on_exit=[load_gripper_controller],
            )
        ),
    ])