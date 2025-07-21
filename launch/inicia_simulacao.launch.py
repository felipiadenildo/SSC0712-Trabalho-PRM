import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node

def generate_launch_description():
    """
    Gera a descrição de lançamento para iniciar o ambiente de simulação no Gazebo,
    baseado na configuração original funcional.
    """

    # ===================================================================================
    # === 1. CONFIGURAÇÃO PRINCIPAL (EDITÁVEL) ===
    # ===================================================================================

    # Flag para controlar a execução do Gazebo com ou sem interface gráfica (GUI).
    # 'false' (padrão): Inicia com a GUI para visualização e depuração.
    # 'true': Inicia em modo headless, apenas o servidor de física, para testes ou para poupar recursos.
    headless_mode = 'true'

    # ===================================================================================
    # === 2. DECLARAÇÃO DE ARGUMENTOS E CAMINHOS ===
    # ===================================================================================
    
    # Argumento para permitir a alteração do ficheiro de mundo via linha de comando.
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='arena_cilindros.sdf',
        description='Nome do ficheiro .sdf do mundo a ser carregado (deve estar na pasta /world).'
    )

    # Encontra o diretório de partilha do pacote 'prm' para localizar os ficheiros.
    pkg_prm_share = get_package_share_directory('prm')

    # Constrói o caminho completo para o ficheiro de mundo selecionado.
    world_path = PathJoinSubstitution([
        pkg_prm_share,
        'world',
        LaunchConfiguration('world')
    ])

    # ===================================================================================
    # === 3. CONFIGURAÇÃO DE AMBIENTE DO GAZEBO (LÓGICA DA VERSÃO ORIGINAL) ===
    # ===================================================================================

    # A sua versão original funcionava devido a esta configuração precisa.
    # O Gazebo precisa que estas variáveis de ambiente sejam definidas para encontrar
    # os modelos 3D (.dae, .stl) e os plugins de simulação (.so).

    # Define o caminho para os modelos 3D.
    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(pkg_prm_share, 'models')
    )

    # Cria um dicionário com o caminho para os plugins, que será passado
    # diretamente para o processo do Gazebo.
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.environ.get('LD_LIBRARY_PATH', '')
    }

    # ===================================================================================
    # === 4. INICIALIZAÇÃO DO SIMULADOR GAZEBO (USANDO 'ign gazebo') ===
    # ===================================================================================

    # Inicia o SERVIDOR do Gazebo (ign gazebo -s).
    # Responsável por toda a simulação de física e lógica.
    start_gazebo_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', '-v', '3', world_path],
        output='screen',
        additional_env=gz_env, # Passa as variáveis de ambiente para o processo
    )

    # Inicia o CLIENTE do Gazebo (ign gazebo -g), que é a interface gráfica (GUI).
    # Este processo só é executado se a variável headless_mode no topo for 'false'.
    start_gazebo_client = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g', '-v', '3'],
        output='screen',
        additional_env=gz_env,
        condition=IfCondition(PythonExpression(["'", headless_mode, "' == 'false'"]))
    )

    # ===================================================================================
    # === 5. PONTE DE COMUNICAÇÃO ROS 2 <-> GAZEBO ===
    # ===================================================================================

    # Cria uma ponte para o tópico /clock, essencial para a sincronização de tempo.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_clock",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    # ===================================================================================
    # === 6. MONTAGEM DA DESCRIÇÃO FINAL DO LANÇAMENTO ===
    # ===================================================================================
    
    return LaunchDescription([
        # Argumentos
        declare_world_arg,
        
        # Configuração do Ambiente (Lógica da Versão Original)
        gz_resource_path,

        # Processos do Gazebo
        start_gazebo_server,
        start_gazebo_client,

        # Ponte de Comunicação
        clock_bridge,
    ])
