# -*- coding: utf-8 -*-

# Nome do Arquivo: robo_mapper.py
#
# Descrição:
# Este nó é responsável por criar um mapa de ocupação 2D em tempo real.
# Ele utiliza os dados de um scanner a laser (Lidar) para detectar obstáculos
# e a odometria para saber a posição do robô no mundo. A principal técnica
# empregada é o "ray casting", que marca o espaço livre e os obstáculos,
# construindo um mapa que pode ser usado por um nó de planejamento de caminho.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
import numpy as np
import math
from typing import Optional

# Importações de Mensagens ROS
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

class RoboMapperNode(Node):
    """
    Nó que constrói e publica um mapa de ocupação 2D a partir dos
    dados do Lidar e da odometria do robô.
    """
    def __init__(self):
        """Construtor da classe RoboMapperNode."""
        super().__init__('robo_mapper')

        # --- Parâmetros ---
        self._declare_parameters()
        self._load_parameters()

        # --- Mapa ---
        # Inicializa a matriz do mapa com o valor -1 (desconhecido).
        self.map_data = np.full((self.p_map_height, self.p_map_width), -1, dtype=np.int8)

        # Calcula a posição da origem do mapa no mundo para que o centro do mapa
        # coincida com a origem do mundo (0,0).
        self.map_origin_x = -self.p_map_width / 2.0 * self.p_map_resolution
        self.map_origin_y = -self.p_map_height / 2.0 * self.p_map_resolution

        # --- Variáveis de Estado ---
        # Armazenam a última pose conhecida do robô.
        self.robot_pose_x: Optional[float] = None
        self.robot_pose_y: Optional[float] = None
        self.robot_pose_theta: Optional[float] = None

        # --- Qualidade de Serviço (QoS) para o Mapa ---
        # É VITAL que o mapa seja publicado com durabilidade TRANSIENT_LOCAL.
        # Isso garante que qualquer nó que se conecte ao tópico /map a qualquer momento
        # receberá imediatamente o último mapa publicado, sem precisar esperar por uma nova publicação.
        qos_map_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publisher e Subscribers ---
        self.map_publisher = self.create_publisher(OccupancyGrid, self.p_map_topic, qos_map_profile)
        self.create_subscription(Odometry, self.p_odom_topic, self.odom_callback, 10)
        self.create_subscription(LaserScan, self.p_scan_topic, self.scan_callback, 10)

        # --- Timer de Publicação ---
        # Publica o mapa periodicamente, em vez de a cada atualização, para economizar recursos.
        self.timer = self.create_timer(1.0 / self.p_publish_rate, self.publish_map)

        self.get_logger().info(f"✅ Robo Mapper iniciado. Mapa (LxA): {self.p_map_width}x{self.p_map_height} células, Res: {self.p_map_resolution} m/célula.")
        self.get_logger().info(f"Publicando mapa no tópico '{self.p_map_topic}' a {self.p_publish_rate} Hz.")

    def _declare_parameters(self):
        """Declara todos os parâmetros ROS para este nó."""
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom_gt')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_resolution', 0.05)  # 5 cm por célula
        self.declare_parameter('map_width', 400)        # Largura em células (400 * 0.05 = 20 metros)
        self.declare_parameter('map_height', 400)       # Altura em células (400 * 0.05 = 20 metros)
        self.declare_parameter('publish_rate', 0.5)     # Publicar mapa a cada 2 segundos (0.5 Hz)

    def _load_parameters(self):
        """Carrega os parâmetros ROS para variáveis de instância."""
        self.p_map_topic = self.get_parameter('map_topic').value
        self.p_odom_topic = self.get_parameter('odom_topic').value
        self.p_scan_topic = self.get_parameter('scan_topic').value
        self.p_map_resolution = self.get_parameter('map_resolution').value
        self.p_map_width = self.get_parameter('map_width').value
        self.p_map_height = self.get_parameter('map_height').value
        self.p_publish_rate = self.get_parameter('publish_rate').value

    def odom_callback(self, msg: Odometry):
        """Callback que atualiza a pose (posição e orientação) do robô."""
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Converte a orientação de Quaternião para um ângulo de Euler (yaw)
        self.robot_pose_theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def scan_callback(self, msg: LaserScan):
        """
        Callback principal que processa os dados do Lidar para atualizar o mapa.
        """
        # Verificar se houve mudanças significativas
        old_map = self.map_data.copy()
        if self.robot_pose_x is None:
            # Não podemos mapear sem saber onde o robô está.
            return

        # Converte a posição atual do robô para coordenadas do grid.
        robot_grid_x, robot_grid_y = self.world_to_grid(self.robot_pose_x, self.robot_pose_y)
        if robot_grid_x is None: return

        # Itera sobre cada raio do Lidar
        for i, r in enumerate(msg.ranges):
            # Ignora leituras inválidas do sensor
            if not (msg.range_min < r < msg.range_max):
                continue

            # Calcula o ângulo real do raio atual
            angle = msg.angle_min + i * msg.angle_increment
            
            # Calcula a posição (x, y) no mundo onde o raio atingiu um objeto
            world_x = self.robot_pose_x + r * math.cos(self.robot_pose_theta + angle)
            world_y = self.robot_pose_y + r * math.sin(self.robot_pose_theta + angle)
            
            # Converte as coordenadas do ponto de impacto para o grid do mapa
            hit_grid_x, hit_grid_y = self.world_to_grid(world_x, world_y)
            if hit_grid_x is None: continue

            # *** A MELHORIA FUNDAMENTAL: RAY CASTING ***
            # Traça uma linha da posição do robô até o ponto de impacto,
            # marcando as células no caminho como LIVRES.
            self._ray_cast(robot_grid_x, robot_grid_y, hit_grid_x, hit_grid_y)

            # Marca a célula final (o ponto de impacto) como OCUPADA.
            if 0 <= hit_grid_x < self.p_map_width and 0 <= hit_grid_y < self.p_map_height:
                self.map_data[hit_grid_y, hit_grid_x] = 100

    def world_to_grid(self, world_x: float, world_y: float) -> Optional[tuple[int, int]]:
        """Converte coordenadas do mundo (metros) para coordenadas do grid (células)."""
        grid_x = int((world_x - self.map_origin_x) / self.p_map_resolution)
        grid_y = int((world_y - self.map_origin_y) / self.p_map_resolution)
        
        # Verifica se as coordenadas estão dentro dos limites do mapa
        if 0 <= grid_x < self.p_map_width and 0 <= grid_y < self.p_map_height:
            return grid_x, grid_y
        return None

    def _ray_cast(self, x0: int, y0: int, x1: int, y1: int):
        """
        Implementação do algoritmo de linha de Bresenham para traçar um raio no grid.
        Marca todas as células ao longo da linha (exceto a última) como livres (0).
        """
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        while True:
            # Marca a célula atual como livre, se estiver dentro dos limites
            if 0 <= x0 < self.p_map_width and 0 <= y0 < self.p_map_height:
                self.map_data[y0, x0] = 0
            
            if x0 == x1 and y0 == y1:
                break
            
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
    
    def publish_map(self):
        """Cria e publica a mensagem OccupancyGrid com os dados atuais do mapa."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map' # O frame de referência para o mapa

        # Preenche os metadados do mapa
        msg.info.resolution = self.p_map_resolution
        msg.info.width = self.p_map_width
        msg.info.height = self.p_map_height
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.orientation.w = 1.0 # Orientação padrão

        # Converte a matriz 2D numpy para uma lista 1D, como o formato da mensagem exige.
        # O método `ravel()` é um pouco mais eficiente que `flatten()` pois não cria uma cópia se possível.
        msg.data = self.map_data.ravel().tolist()
        
        self.map_publisher.publish(msg)
        self.get_logger().info('Mapa atualizado e publicado.', throttle_duration_sec=5)
        msg.info.map_load_time = msg.header.stamp

def main(args=None):
    """Função principal que inicializa e executa o nó."""
    rclpy.init(args=args)
    mapper_node = RoboMapperNode()
    try:
        rclpy.spin(mapper_node)
    except KeyboardInterrupt:
        pass
    finally:
        mapper_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()