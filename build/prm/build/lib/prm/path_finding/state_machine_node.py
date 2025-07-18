# prm/path_finding/state_machine_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from .state_machine import State
import tf_transformations
import math
import numpy as np

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        # --- Variáveis de Classe ---
        self.state = State.SEARCHING_FLAG
        self.robot_pose = None
        self.robot_orientation = None
        self.base_pose = None
        self.laser_data = None
        
        # Novas variáveis para seguir o caminho
        self.current_path = []
        self.path_target_index = 0

        # --- Publishers ---
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # Novo publisher para enviar o alvo para o planejador
        self.goal_pub_ = self.create_publisher(PoseStamped, '/robot/goal_pose', 10)
        
        # --- Subscribers ---
        self.odom_sub_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub_ = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        # Novo subscriber para receber o caminho do planejador
        self.path_sub_ = self.create_subscription(Path, '/robot/path', self.path_callback, 10)
        
        # --- Timer ---
        self.timer_ = self.create_timer(0.1, self.execute_state_machine)
        
        self.twist_msg = Twist()
        self.get_logger().info('Nó da Máquina de Estados iniciado.')

    def odom_callback(self, msg: Odometry):
        # Atualiza a pose atual do robô
        current_pose = msg.pose.pose
        self.robot_pose = current_pose
        _, _, self.robot_orientation = tf_transformations.euler_from_quaternion(
            [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        )
        # Salva a pose da base na primeira chamada
        if self.base_pose is None:
            self.base_pose = current_pose
            self.get_logger().info(f"Base salva na posição: x={self.base_pose.position.x:.2f}, y={self.base_pose.position.y:.2f}")

    def laser_callback(self, msg: LaserScan):
        self.laser_data = msg.ranges

    def path_callback(self, msg: Path):
        """ Callback acionado quando um novo caminho é recebido do planejador. """
        if self.state != State.FOLLOWING_PATH:
            self.get_logger().info(f'Novo caminho com {len(msg.poses)} waypoints recebido!')
            # Armazena os pontos (poses) do caminho
            self.current_path = [pose.pose.position for pose in msg.poses]
            self.path_target_index = 0
            # Muda para o estado de seguir o caminho
            self.state = State.FOLLOWING_PATH

    def publish_goal(self, goal_point: Point):
        """ Publica uma meta para o planejador de caminho. """
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map' # ou 'odom', dependendo do seu frame de referência
        goal_msg.pose.position = goal_point
        self.goal_pub_.publish(goal_msg)
        self.get_logger().info(f"Publicando novo alvo: x={goal_point.x:.2f}, y={goal_point.y:.2f}")

    def follow_path(self):
        """ Lógica para seguir a lista de waypoints. """
        if not self.current_path or self.robot_pose is None:
            return

        # Pega o waypoint alvo atual
        target_point = self.current_path[self.path_target_index]
        robot_pos = self.robot_pose.position
        
        # Calcula a distância e o ângulo para o waypoint
        distance_to_target = math.sqrt((target_point.x - robot_pos.x)**2 + (target_point.y - robot_pos.y)**2)
        angle_to_target = math.atan2(target_point.y - robot_pos.y, target_point.x - robot_pos.x)
        
        # Calcula o erro de ângulo
        angle_error = angle_to_target - self.robot_orientation
        # Normaliza o erro para o intervalo [-pi, pi]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Controlador Proporcional Simples
        linear_speed = 0.3
        angular_speed = 0.8 * angle_error

        # Se o robô estiver desalinhado, ele primeiro gira e depois anda
        if abs(angle_error) > 0.2:
            linear_speed = 0.0

        self.twist_msg.linear.x = linear_speed
        self.twist_msg.angular.z = angular_speed
        self.cmd_vel_pub_.publish(self.twist_msg)

        # Verifica se o waypoint foi alcançado
        if distance_to_target < 0.2: # Tolerância de 20cm
            self.path_target_index += 1
            # Verifica se o caminho terminou
            if self.path_target_index >= len(self.current_path):
                self.get_logger().info("Fim do caminho alcançado!")
                self.current_path = []
                self.twist_msg.linear.x = 0.0
                self.twist_msg.angular.z = 0.0
                self.cmd_vel_pub_.publish(self.twist_msg)
                # Mude para o próximo estado apropriado aqui!
                # Ex: self.state = State.ALIGNING_FOR_CAPTURE
                self.state = State.DONE # Por enquanto, vamos para DONE
            else:
                self.get_logger().info(f"Waypoint {self.path_target_index-1} alcançado. Indo para o próximo.")

    def execute_state_machine(self):
        """ Lógica principal que roda em um timer. """
        if self.robot_pose is None or self.laser_data is None:
            self.get_logger().info('Aguardando dados de odometria e laser...')
            return
        
        # --- Lógica de Transição de Estados ---
        if self.state == State.SEARCHING_FLAG:
            # Simplesmente gira até que alguma lógica de visão encontre a bandeira
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.3
            self.cmd_vel_pub_.publish(self.twist_msg)
            # **Lógica de Transição (PLACEHOLDER)**: 
            # Quando a visão encontrar a bandeira, mude para MOVING_TO_FLAG
            # Por agora, vamos simular que encontramos a bandeira após 5s
            if self.get_clock().now().nanoseconds / 1e9 > 5.0:
                 self.state = State.MOVING_TO_FLAG

        elif self.state == State.MOVING_TO_FLAG:
            # A única tarefa deste estado é pedir o caminho para a bandeira.
            # **NOTA**: Você precisará obter as coordenadas da bandeira da sua visão.
            # Por enquanto, vamos usar um valor fixo.
            flag_position = Point(x=5.0, y=0.0, z=0.0)
            self.publish_goal(flag_position)
            # Fica "parado" esperando o path_callback mudar o estado para FOLLOWING_PATH
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.cmd_vel_pub_.publish(self.twist_msg)

        elif self.state == State.FOLLOWING_PATH:
            # A lógica de movimento agora está encapsulada aqui
            self.follow_path()
        
        elif self.state == State.DONE:
            # Missão concluída, para o robô.
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.cmd_vel_pub_.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    state_machine_node = StateMachineNode()
    rclpy.spin(state_machine_node)
    state_machine_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()