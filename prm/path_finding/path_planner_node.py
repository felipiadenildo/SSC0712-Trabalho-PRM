# -*- coding: utf-8 -*-

# Nome do Arquivo: path_planner_node.py
#
# Descrição:
# Este nó ROS2 é o cérebro do planejamento de caminho do robô. Ele é responsável por:
# 1. Receber e processar um mapa de ocupação do ambiente.
# 2. Conhecer a posição atual do robô através da odometria.
# 3. Receber uma pose de destino (objetivo).
# 4. Utilizar o algoritmo A* para calcular o caminho mais curto e seguro
#    entre a posição atual e o destino, evitando obstáculos.
# 5. Publicar o caminho encontrado para que outros nós (como o de controle)
#    possam executá-lo.
#
# Objetivos do Projeto Atendidos:
# - Locate the flag (ao receber a localização da bandeira como um 'goal')
# - Return to base (ao receber a localização da base como um 'goal')
# - Avoid collision (através do uso do mapa de ocupação e do A* com raio de segurança)

import rclpy
from rclpy.node import Node
import numpy as np

# Importações de Mensagens ROS
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

# Importações de Qualidade de Serviço (QoS)
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

# Importa a implementação do algoritmo A* de um arquivo local
from .a_star import AStar

class PathPlannerNode(Node):
    """
    Nó de planejamento de caminho que calcula uma rota usando o algoritmo A*
    baseado em um mapa de ocupação e publica essa rota.
    """

    def __init__(self):
        """Construtor da classe PathPlannerNode."""
        super().__init__('path_planner_node')

        # --- Declaração de Parâmetros ---
        # Declarar parâmetros permite que eles sejam configurados externamente (ex: via launch files).
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom_gt')
        self.declare_parameter('goal_topic', '/robot/goal_pose')
        self.declare_parameter('path_topic', '/robot/path')
        self.declare_parameter('safety_radius_cells', 5) # Raio de segurança em células do grid

        # --- Obtenção dos Parâmetros ---
        # Atribui os valores dos parâmetros a variáveis de instância para fácil acesso.
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.safety_radius = self.get_parameter('safety_radius_cells').get_parameter_value().integer_value
        
        self.get_logger().info(f"Usando raio de segurança de {self.safety_radius} células.")

        # --- Perfis de Qualidade de Serviço (QoS) ---
        # Perfil para publishers e subscribers que lidam com dados que não podem ser perdidos.
        qos_reliable_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Perfil específico para o mapa e o alvo (goal).
        # TRANSIENT_LOCAL garante que novos nós que se conectam recebam a última mensagem publicada.
        # Essencial para que o planner receba o mapa e o alvo, mesmo que tenham sido publicados antes de ele iniciar.
        qos_transient_local_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publishers ---
        # Publica o caminho calculado para que outros nós possam usá-lo.
        self.path_publisher = self.create_publisher(Path, path_topic, qos_reliable_profile)

        # --- Subscribers ---
        # Se inscreve no tópico do mapa para receber os dados do ambiente.
        self.create_subscription(OccupancyGrid, map_topic, self.map_callback, qos_transient_local_profile)
        # Se inscreve no tópico de odometria para saber a posição atual do robô.
        self.create_subscription(Odometry, odom_topic, self.odom_callback, qos_reliable_profile)
        # Se inscreve no tópico do alvo para iniciar o planejamento quando um novo objetivo é recebido.
        self.create_subscription(PoseStamped, goal_topic, self.goal_callback, qos_transient_local_profile)

        # --- Variáveis de Instância ---
        # Armazenam o estado mais recente recebido pelos subscribers.
        self.robot_pose = None
        self.map_data = None
        self.map_info = None
        self.a_star = None # Objeto do planejador A*, inicializado após receber o mapa.

        self.get_logger().info("✅ Nó Path Planner iniciado. Aguardando mapa e odometria...")

    def map_callback(self, msg: OccupancyGrid):
        """
        Callback executado sempre que um novo mapa de ocupação é recebido.
        Processa e armazena os dados do mapa para uso no planejamento.
        """
        if self.map_data is not None:
            # Evita reprocessar o mesmo mapa se ele for estático.
            # Pode ser removido se o mapa for dinâmico.
            return

        self.get_logger().info('🗺️ Mapa recebido e processado!')
        self.map_info = msg.info  # Armazena todas as informações do mapa (resolução, origem, etc.)
        
        # Converte o array 1D de dados do mapa em uma matriz 2D (Numpy) para facilitar o acesso.
        # A ordem (height, width) é a convenção correta.
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((self.map_info.height, self.map_info.width))

        # Inicializa a classe A* com o mapa pronto e o raio de segurança.
        self.a_star = AStar(self.map_data, safety_radius_cells=self.safety_radius)
        self.get_logger().info(f"Mapa processado com dimensões {self.map_data.shape}. A* está pronto!")

    def odom_callback(self, msg: Odometry):
        """Callback para armazenar a pose (posição e orientação) mais recente do robô."""
        self.robot_pose = msg.pose.pose

    def goal_callback(self, msg: PoseStamped):
        """
        Callback principal que aciona o planejamento de caminho quando um novo alvo é recebido.
        """
        if self.a_star is None or self.robot_pose is None:
            self.get_logger().warn('🎯 Alvo recebido, mas o mapa ou a odometria ainda não estão prontos. Aguardando...')
            return

        self.get_logger().info("🎯 Novo alvo recebido! Iniciando o planejamento de caminho.")

        # --- Coordenadas de Início e Fim ---
        start_world = self.robot_pose.position
        goal_world = msg.pose.position

        # Converte as coordenadas do mundo (metros) para coordenadas do grid (células/pixels).
        start_grid = self.world_to_grid(start_world.x, start_world.y)
        goal_grid = self.world_to_grid(goal_world.x, goal_world.y)

        # --- Validação dos Pontos ---
        if not self.is_valid_point(start_grid, "inicial"):
            return
        if not self.is_valid_point(goal_grid, "final"):
            return

        self.get_logger().info(f"Planejando de {start_grid} (mundo: [{start_world.x:.2f}, {start_world.y:.2f}]) para {goal_grid} (mundo: [{goal_world.x:.2f}, {goal_world.y:.2f}])")

        # --- Execução do A* ---
        path_grid = self.a_star.find_path(start_grid, goal_grid)

        if path_grid:
            self.get_logger().info(f"✅ Caminho encontrado com {len(path_grid)} pontos. Publicando...")
            self.publish_path(path_grid, msg.header.frame_id)
        else:
            self.get_logger().warn("⚠️ Não foi possível encontrar um caminho para o alvo especificado.")

    def publish_path(self, path_grid, frame_id: str):
        """
        Converte um caminho de coordenadas de grid para o mundo e o publica como uma mensagem Path.
        """
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id  # Geralmente 'map' ou 'odom'

        for point_grid in path_grid:
            # Converte cada ponto do caminho de volta para coordenadas do mundo.
            world_coords = self.grid_to_world(point_grid[0], point_grid[1])
            
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x = world_coords[0]
            pose.pose.position.y = world_coords[1]
            # A orientação (quaternion) pode ser deixada como padrão (0,0,0,1)
            # ou calculada para apontar para o próximo ponto no caminho.
            # Por simplicidade, deixamos como padrão.
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
            
        self.path_publisher.publish(path_msg)
        self.get_logger().info("Caminho publicado com sucesso!")

    # --- Funções Utilitárias de Conversão e Validação ---

    def world_to_grid(self, world_x: float, world_y: float) -> tuple[int, int] | None:
        """Converte coordenadas do mundo (metros) para coordenadas do grid (células)."""
        if self.map_info is None:
            self.get_logger().error("Tentativa de conversão de coordenadas sem informações do mapa.")
            return None
        
        # A conversão subtrai a origem do mapa e divide pela resolução.
        grid_x = int((world_x - self.map_info.origin.position.x) / self.map_info.resolution)
        grid_y = int((world_y - self.map_info.origin.position.y) / self.map_info.resolution)
        
        return (grid_x, grid_y)

    def grid_to_world(self, grid_x: int, grid_y: int) -> tuple[float, float] | None:
        """
        Converte coordenadas do grid (células) de volta para coordenadas do mundo (metros).
        Retorna o centro da célula para um movimento mais suave.
        """
        if self.map_info is None:
            return None

        # Adiciona metade da resolução para obter o centro da célula.
        world_x = (grid_x * self.map_info.resolution) + self.map_info.origin.position.x + (self.map_info.resolution / 2.0)
        world_y = (grid_y * self.map_info.resolution) + self.map_info.origin.position.y + (self.map_info.resolution / 2.0)
        
        return (world_x, world_y)
        
    def is_valid_point(self, grid_point: tuple[int, int], point_type: str) -> bool:
        """
        Verifica se um ponto no grid é válido (dentro dos limites e não é um obstáculo).
        """
        if grid_point is None:
            self.get_logger().error(f"Coordenada {point_type} é Nula.")
            return False

        grid_x, grid_y = grid_point
        
        # Verifica se está dentro dos limites do mapa.
        if not (0 <= grid_x < self.map_info.width and 0 <= grid_y < self.map_info.height):
            self.get_logger().error(f"Posição {point_type} ({grid_x}, {grid_y}) está fora dos limites do mapa. Abortando.")
            return False
        
        # Verifica se a célula é um obstáculo.
        # Um valor de 100 significa obstáculo definitivo.
        # Usamos um limiar (> 50) para sermos conservadores e evitarmos áreas incertas.
        if self.map_data[grid_y, grid_x] > 50:
            self.get_logger().error(f"Posição {point_type} ({grid_x}, {grid_y}) está sobre um obstáculo. Abortando.")
            return False
            
        return True


def main(args=None):
    """Função principal que inicializa e executa o nó."""
    rclpy.init(args=args)
    path_planner_node = PathPlannerNode()
    try:
        rclpy.spin(path_planner_node)
    except KeyboardInterrupt:
        pass  # Permite que o programa saia com Ctrl+C sem erro
    finally:
        # Destrói o nó e desliga o rclpy de forma limpa.
        path_planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()