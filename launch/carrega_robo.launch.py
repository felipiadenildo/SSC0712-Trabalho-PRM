# -*- coding: utf-8 -*-

# Nome do Arquivo: carrega_robo.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import subprocess

# Adicione esta função antes do generate_launch_description()
def fix_permissions():
    try:
        subprocess.run(["sudo", "chmod", "a+rw", "/dev/dri/renderD128"], check=True)
        subprocess.run(["sudo", "chmod", "a+rw", "/dev/dri/renderD129"], check=True)
        print("✅ Permissões de renderização corrigidas")
    except Exception as e:
        print(f"⚠️ Erro ao corrigir permissões: {e}")

def generate_launch_description():
    """
    Inicia o robô PRM na simulação. Esta versão combina a lógica funcional do arquivo
    original com a flexibilidade de argumentos de lançamento e código organizado.
    """
    fix_permissions()

    # --- 1. ARGUMENTOS DE LANÇAMENTO ---
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Usar o tempo da simulação (essencial para o Gazebo).",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Controla se o RViz2 deve ser iniciado.",
        ),
        DeclareLaunchArgument(
            "spawn_x", default_value="-8.0", description="Posição inicial X do robô."
        ),
        DeclareLaunchArgument(
            "spawn_y", default_value="-0.5", description="Posição inicial Y do robô."
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    # --- 2. CAMINHOS E CONFIGURAÇÕES ---
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("prm"), "description", "robot.urdf.xacro"]
    )
    # Comando 'xacro' restaurado para a sintaxe original funcional (com espaço)
    robot_urdf_content = Command(["xacro ", urdf_path])
    
    controller_params_path = PathJoinSubstitution(
        [FindPackageShare("prm"), "config", "controller_config.yaml"]
    )
    
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("prm"), "rviz", "rviz_config.rviz"]
    )
    
    rviz_config_path2 = PathJoinSubstitution(
        [FindPackageShare("prm"), "rviz", "urdf.rviz"]
    )
    
    # --- 3. DEFINIÇÃO DOS NÓS E PROCESSOS (BASEADO NO ORIGINAL) ---

    # Nó do publicador de estados do robô
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_urdf_content},
            {"use_sim_time": use_sim_time},
        ],
    )

    # Spawn do robô no Gazebo
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "prm_robot",
            "-topic", "robot_description",
            "-z", "0.4",
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
        ],
    )

    # Ponte de comunicação ROS <-> Gazebo (CONFIGURAÇÃO EXATA DO ORIGINAL)
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_prm_robot",
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            # Tópicos da câmera restaurados para os do arquivo original
            "/robot_cam/colored_map@sensor_msgs/msg/Image[gz.msgs.Image",
            "/robot_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",            
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/prm_robot/pose@geometry_msgs/msg/Pose[gz.msgs.Pose",
        ],
        remappings=[("/model/prm_robot/pose", "/odom_raw")],
        output="screen",
    )

    # Controladores (ros2_control)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
        output="screen"
    )

    start_diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller"],
        parameters=[controller_params_path],
        output="screen",
    )

    start_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        parameters=[controller_params_path],
        output="screen",
    )
    
    # Nós da Aplicação PRM
    odom_gt_node = Node(
        package="prm",
        executable="ground_truth_odometry",
        name="odom_gt_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    robo_mapper_node = Node(
        package="prm",
        executable="robo_mapper",
        name="mapper",
        output='screen',
        # parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # Utilitário de redirecionamento de tópico (restaurado do original)
    relay_cmd_vel = Node(
        name="relay_cmd_vel",
        package="topic_tools",
        executable="relay",
        parameters=[{
            "input_topic": "/cmd_vel",
            "output_topic": "/diff_drive_base_controller/cmd_vel_unstamped",
        }],
        output="screen",
    )

    # Visualização com RViz
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            # parameters=[{'use_sim_time': use_sim_time}]
        )
    
    # Visualização com RViz
    rviz_node_2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path2],
            # parameters=[{'use_sim_time': use_sim_time}]
        )
    
    robot_path_node= Node(
            package='prm',
            executable='robot_path_node',   # o nome que você registrou no setup.py
            name='robot_path_node',
            output='screen',
        )
    
    # --- 4. ORDEM DE EXECUÇÃO E EVENTOS ---
    return LaunchDescription(declared_arguments + [
        # Nós que podem iniciar em paralelo
        bridge_node,
        robot_state_publisher_node,
        spawn_entity_node,
        odom_gt_node,
        robo_mapper_node,
        relay_cmd_vel,
        rviz_node,
        rviz_node_2,
        robot_path_node,
        
        # Handlers que garantem a sequência correta de carregamento dos controladores
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=spawn_entity_node, on_exit=[load_joint_state_broadcaster]),
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=load_joint_state_broadcaster, on_exit=[start_diff_drive_controller]),
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=start_diff_drive_controller, on_exit=[start_gripper_controller]),
        ),
        
        
    ])