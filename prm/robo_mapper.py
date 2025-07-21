import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
import numpy as np
import math

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan

class RoboMapperNode(Node):
    """
    Nó responsável por criar um mapa de ocupação 2D a partir dos dados
    do Lidar e da odometria do robô.
    """
    def __init__(self):
        super().__init__('robo_mapper')

        # --- Qualidade de Serviço (QoS) para o Mapa ---
        # Isto é MUITO IMPORTANTE. O mapa é um tópico de dados estáticos.
        # A durabilidade "TRANSIENT_LOCAL" garante que qualquer nó que se subscreva
        # ao tópico /map receberá a última mensagem publicada, mesmo que o nó
        # se tenha subscrito depois da publicação.
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publisher ---
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', qos_profile)

        # --- Subscribers ---
        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # --- Configurações do Mapa ---
        self.map_resolution = 0.05  # 5 cm por célula
        self.map_width = 200        # Largura em células (200 * 0.05 = 10 metros)
        self.map_height = 200       # Altura em células (200 * 0.05 = 10 metros)
        
        # Inicializa a grelha do mapa.
        # -1: Desconhecido, 0: Livre, 100: Ocupado
        self.map_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)

        # Posição da origem do mapa no mundo
        self.map_origin_x = -self.map_width / 2.0 * self.map_resolution
        self.map_origin_y = -self.map_height / 2.0 * self.map_resolution
        
        # Variáveis para a posição do robô
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Timer para publicar o mapa periodicamente
        self.timer = self.create_timer(2.0, self.publish_map) # Publica o mapa a cada 2 segundos

        self.get_logger().info('Robo Mapper iniciado. Construindo o mapa...')

    def odom_callback(self, msg: Odometry):
        """Atualiza a posição e orientação do robô."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Converte o quaternion para um ângulo de Euler (yaw)
        self.robot_theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def scan_callback(self, msg: LaserScan):
        """Processa os dados do Lidar para atualizar o mapa."""
        angle = msg.angle_min
        for r in msg.ranges:
            # Ignora leituras inválidas (infinitas ou muito curtas)
            if r > msg.range_min and r < msg.range_max:
                # Calcula a posição (x, y) do ponto detetado pelo Lidar no referencial do mundo
                world_x = self.robot_x + r * math.cos(self.robot_theta + angle)
                world_y = self.robot_y + r * math.sin(self.robot_theta + angle)
                
                # Converte as coordenadas do mundo para as coordenadas da grelha do mapa
                grid_x = int((world_x - self.map_origin_x) / self.map_resolution)
                grid_y = int((world_y - self.map_origin_y) / self.map_resolution)

                # Marca a célula como ocupada (100) se estiver dentro dos limites do mapa
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                    self.map_data[grid_y, grid_x] = 100
            
            angle += msg.angle_increment

    def publish_map(self):
        """Cria e publica a mensagem OccupancyGrid."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Informações do mapa
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        
        # Converte a matriz 2D numpy para uma lista 1D, como o formato da mensagem exige
        msg.data = self.map_data.flatten().tolist()
        
        self.map_publisher.publish(msg)
        self.get_logger().info('Mapa atualizado e publicado.')

def main(args=None):
    rclpy.init(args=args)
    mapper_node = RoboMapperNode()
    rclpy.spin(mapper_node)
    mapper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

