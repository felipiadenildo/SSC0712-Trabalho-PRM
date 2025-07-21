import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from .a_star import AStar

class PathPlannerNode(Node):
    """
    Nó responsável por receber um alvo, planear um caminho usando o A*
    e publicar a rota.
    """
    def __init__(self):
        super().__init__('path_planner_node')

        # --- Publisher ---
        self.path_publisher = self.create_publisher(Path, '/robot/path', 10)

        # --- Subscribers ---
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        # self.create_subscription(PoseStamped, '/robot/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)

        # --- Variáveis de Instância ---
        self.robot_pose = None
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None
        self.a_star = None

        self.get_logger().info("Path Planner Node foi iniciado e está aguardando o mapa.")
        
        # --- PERFIL DE QUALIDADE DE SERVIÇO PARA O ALVO ---
        qos_profile_goal = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.create_subscription(PoseStamped, '/robot/goal_pose', self.goal_callback, qos_profile_goal)

    def map_callback(self, msg: OccupancyGrid):
        """Callback que processa e armazena o mapa de ocupação."""
        self.get_logger().info('Mapa recebido!')
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))
        
        # Inicializa a classe A* com o mapa pronto
        safety_radius_in_cells = 5
        self.a_star = AStar(self.map_data, safety_radius_cells=safety_radius_in_cells)
        self.get_logger().info(f"Mapa processado. Dimensões: {self.map_data.shape}. A* inicializado.")

    def odom_callback(self, msg: Odometry):
        """Callback para armazenar a pose mais recente do robô."""
        self.robot_pose = msg.pose.pose
    
    # --- FUNÇÕES DE CONVERSÃO DE COORDENADAS (CORRIGIDAS) ---

    def world_to_grid(self, world_x, world_y):
        """Converte coordenadas do mundo (metros) para coordenadas da grelha (píxeis)."""
        if self.map_origin is None or self.map_resolution is None:
            self.get_logger().error("Tentativa de conversão de coordenadas sem mapa ou resolução.")
            return None
        
        # Fórmula correta para a conversão
        grid_x = int((world_x - self.map_origin.position.x) / self.map_resolution)
        grid_y = int((world_y - self.map_origin.position.y) / self.map_resolution)
        
        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y):
        """Converte coordenadas da grelha (píxeis) de volta para coordenadas do mundo (metros)."""
        world_x = (grid_x * self.map_resolution) + self.map_origin.position.x
        world_y = (grid_y * self.map_resolution) + self.map_origin.position.y
        
        return (world_x, world_y)
        
    def is_valid(self, grid_point):
        """Verifica se um ponto na grelha é válido (dentro dos limites e não é um obstáculo)."""
        if grid_point is None: return False
        grid_x, grid_y = grid_point
        
        if not (0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height):
            self.get_logger().warn(f"Coordenada ({grid_x}, {grid_y}) está fora dos limites do mapa (0-{self.map_width-1}, 0-{self.map_height-1}).")
            return False
        
        # O valor 100 significa obstáculo definitivo.
        if self.map_data[grid_y, grid_x] > 50:
            self.get_logger().warn(f"Coordenada ({grid_x}, {grid_y}) está sobre um obstáculo.")
            return False
            
        return True

    def goal_callback(self, msg: PoseStamped):
        """Callback que planeia um caminho da posição atual do robô até o alvo."""
        if self.a_star is None or self.robot_pose is None:
            self.get_logger().warn('Alvo recebido, mas o mapa ou a odometria ainda não estão prontos.')
            return

        self.get_logger().info("Novo alvo recebido! Iniciando planejamento.")
        
        start_world = self.robot_pose.position
        goal_world = msg.pose.position

        start_grid = self.world_to_grid(start_world.x, start_world.y)
        goal_grid = self.world_to_grid(goal_world.x, goal_world.y)
        
        self.get_logger().info(f"Planejando de {start_grid} para {goal_grid}")

        if not self.is_valid(start_grid) or not self.is_valid(goal_grid):
            self.get_logger().error(f"Posição inicial ({start_grid}) ou final ({goal_grid}) é inválida. Abortando planeamento.")
            return

        path_grid = self.a_star.find_path(start_grid, goal_grid)

        if path_grid:
            self.get_logger().info(f"Caminho encontrado com {len(path_grid)} pontos.")
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'

            for point_grid in path_grid:
                world_coords = self.grid_to_world(point_grid[0], point_grid[1])
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = world_coords[0]
                pose.pose.position.y = world_coords[1]
                path_msg.poses.append(pose)
                
            self.path_publisher.publish(path_msg)
            self.get_logger().info("Caminho publicado com sucesso!")
        else:
            self.get_logger().warn("Não foi possível encontrar um caminho para o alvo.")

def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlannerNode()
    rclpy.spin(path_planner_node)
    path_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

