# prm/path_finding/path_planner_node.py

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from .path_planner_util import find_path, convert_occupancy_grid_to_map

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        
        # --- Parâmetros ---
        # Raio de segurança ao redor do robô (em células do mapa)
        self.declare_parameter('safety_radius', 5) 

        # --- Variáveis ---
        self.current_map = None
        self.map_resolution = 0.0
        self.map_origin = None
        
        # --- Subscribers ---
        # Assina o tópico do mapa para ter sempre a visão mais recente do mundo
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',  # Tópico padrão de mapa do SLAM
            self.map_callback,
            rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        )
        
        # Assina um tópico para receber o "alvo" (PoseStamped)
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/robot/goal_pose', # Tópico customizado para receber alvos
            self.goal_callback,
            10
        )
        
        # --- Publishers ---
        # Publica o caminho encontrado como uma mensagem nav_msgs/Path
        self.path_publisher = self.create_publisher(Path, '/robot/path', 10)
        
        self.get_logger().info('Nó Path Planner iniciado. Aguardando mapa e alvo...')

    def map_callback(self, msg: OccupancyGrid):
        """ Callback para armazenar o mapa mais recente. """
        self.get_logger().info('Mapa recebido!')
        self.current_map = convert_occupancy_grid_to_map(msg)
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        self.get_logger().info(f"Dimensões do mapa: {self.current_map.shape}, Resolução: {self.map_resolution:.3f} m/pixel")

    def world_to_map_coords(self, world_point: Point):
        """ Converte coordenadas do mundo (em metros) para coordenadas do mapa (em pixels). """
        if self.map_origin is None or self.map_resolution == 0:
            return None
        
        map_x = int((world_point.x - self.map_origin.position.x) / self.map_resolution)
        map_y = int((world_point.y - self.map_origin.position.y) / self.map_resolution)
        
        # A conversão de y precisa ser invertida devido à origem do mapa numpy
        map_y = self.current_map.shape[0] - 1 - map_y
        
        return (map_y, map_x) # Retorna (linha, coluna)

    def map_to_world_coords(self, map_point):
        """ Converte coordenadas do mapa (pixels) de volta para coordenadas do mundo (metros). """
        map_y, map_x = map_point
        
        # Inverte a conversão de y
        inverted_y = self.current_map.shape[0] - 1 - map_y

        world_x = (map_x * self.map_resolution) + self.map_origin.position.x
        world_y = (inverted_y * self.map_resolution) + self.map_origin.position.y
        
        return Point(x=world_x, y=world_y, z=0.0)

    def goal_callback(self, msg: PoseStamped):
        """ Callback disparado quando um novo alvo é recebido. """
        if self.current_map is None:
            self.get_logger().warn('Alvo recebido, mas o mapa ainda não está disponível.')
            return

        # Pega a posição do robô (ponto de partida) - por enquanto, vamos fixar um valor
        # No futuro, isso virá da odometria
        # NOTA: O ideal é receber o ponto de partida junto com o alvo.
        start_world = Point(x=0.0, y=0.0, z=0.0) # Placeholder!
        goal_world = msg.pose.position

        start_map = self.world_to_map_coords(start_world)
        goal_map = self.world_to_map_coords(goal_world)

        self.get_logger().info(f'Novo alvo recebido. Início (mapa): {start_map}, Fim (mapa): {goal_map}')

        if start_map is None or goal_map is None:
            self.get_logger().error('Não foi possível converter as coordenadas do alvo para o mapa.')
            return
            
        safety_radius = self.get_parameter('safety_radius').get_parameter_value().integer_value

        # Chama a função de busca de caminho
        path_map_coords = find_path(
            self.current_map,
            start=start_map,
            goal=goal_map,
            sz=safety_radius
        )

        if not path_map_coords:
            self.get_logger().warn('Nenhum caminho encontrado para o alvo.')
            return

        self.get_logger().info(f'Caminho encontrado com {len(path_map_coords)} pontos.')
        
        # Converte o caminho (em pixels) de volta para o mundo (em metros) e publica
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map' # O caminho é definido no frame do mapa

        for map_point in path_map_coords:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position = self.map_to_world_coords(map_point)
            path_msg.poses.append(pose)
        
        self.path_publisher.publish(path_msg)
        self.get_logger().info('Caminho publicado no tópico /robot/path.')

def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlannerNode()
    rclpy.spin(path_planner_node)
    path_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()