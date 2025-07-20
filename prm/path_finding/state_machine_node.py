# /scripts/state_machine_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan # Importa o LaserScan
from std_msgs.msg import Float64MultiArray
from .state_machine import State
import tf_transformations
import math
import time

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        # --- Variáveis de Estado e Posição ---
        self.state = State.SEARCHING_FLAG
        self.robot_pose = None
        self.robot_orientation = None
        self.base_pose = None
        self.flag_pixel_pos = None
        self.laser_data = None # Armazena os dados do laser
        
        # Parâmetros da câmera (ajuste se a resolução da sua câmera for diferente)
        self.camera_center_x = 640 / 2 # Ex: para uma câmera de 640x480

        self.current_path = []
        self.path_target_index = 0
        
        # --- Publishers ---
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub_ = self.create_publisher(PoseStamped, '/robot/goal_pose', 10)
        self.gripper_pub_ = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        
        # --- Subscribers ---
        self.odom_sub_ = self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)
        self.flag_vision_sub_ = self.create_subscription(Point, '/vision/flag_position', self.flag_vision_callback, 10)
        self.path_sub_ = self.create_subscription(Path, '/robot/path', self.path_callback, 10)
        # ✅ NOVO: Subscriber para o LaserScan
        self.laser_sub_ = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # --- Timer ---
        self.timer_ = self.create_timer(0.1, self.execute_state_machine)
        
        self.get_logger().info(f'Nó da Máquina de Estados iniciado. Estado inicial: {self.state.name}')

    def odom_callback(self, msg: Odometry):
        # ... (sem alterações) ...
        current_pose = msg.pose.pose
        self.robot_pose = current_pose
        _, _, self.robot_orientation = tf_transformations.euler_from_quaternion(
            [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        )
        if self.base_pose is None:
            self.base_pose = current_pose.position
            self.get_logger().info(f"Base salva na posição: x={self.base_pose.x:.2f}, y={self.base_pose.y:.2f}")

    def flag_vision_callback(self, msg: Point):
        self.flag_pixel_pos = msg

    def laser_callback(self, msg: LaserScan):
        self.laser_data = msg.ranges

    def path_callback(self, msg: Path):
        # ... (sem alterações) ...
        if self.state in [State.MOVING_TO_FLAG, State.RETURNING_TO_BASE]:
            self.get_logger().info(f'Novo caminho com {len(msg.poses)} waypoints recebido!')
            self.current_path = [pose.pose.position for pose in msg.poses]
            self.path_target_index = 0
            self.state = State.FOLLOWING_PATH

    def publish_goal(self, goal_point: Point):
        # ... (sem alterações) ...
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map' 
        goal_msg.pose.position = goal_point
        self.goal_pub_.publish(goal_msg)
        self.get_logger().info(f"Publicando novo alvo para o planejador: x={goal_point.x:.2f}, y={goal_point.y:.2f}")

    def send_gripper_command(self, elevation: float, left_cm: float, right_cm: float):
        # ... (sem alterações) ...
        msg = Float64MultiArray()
        msg.data = [elevation, left_cm, right_cm]
        self.gripper_pub_.publish(msg)
        self.get_logger().info(f'Enviando comando para garra: Haste={elevation:.2f}, Esq={left_cm:.2f}, Dir={right_cm:.2f}')

    def move_robot(self, linear, angular):
        # ... (sem alterações) ...
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub_.publish(twist)

    def execute_state_machine(self):
        if self.robot_pose is None or self.laser_data is None:
            self.get_logger().warn('Aguardando dados de odometria e laser...')
            return
        
        # ==================== MÁQUINA DE ESTADOS (VERSÃO FINAL) ====================

        if self.state == State.SEARCHING_FLAG:
            if self.flag_pixel_pos is None:
                self.move_robot(0.0, 0.4) # Gira se não vê a bandeira
            else:
                # Bandeira foi detectada, centraliza ela na câmera
                error = self.camera_center_x - self.flag_pixel_pos.x
                # Se estiver quase centralizado...
                if abs(error) < 15: # Tolerância de 15 pixels
                    self.move_robot(0.0, 0.0)
                    self.get_logger().info("Bandeira centralizada! Calculando posição...")
                    
                    # ✅ LÓGICA FINAL: Calcula a posição real da bandeira
                    distancia_bandeira = self.laser_data[0] # Laser aponta para frente (0 graus)
                    
                    # Calcula a posição do alvo em relação ao robô
                    alvo_x_rel = distancia_bandeira * math.cos(self.robot_orientation)
                    alvo_y_rel = distancia_bandeira * math.sin(self.robot_orientation)
                    
                    # Calcula a posição global do alvo
                    alvo_x_global = self.robot_pose.position.x + alvo_x_rel
                    alvo_y_global = self.robot_pose.position.y + alvo_y_rel
                    
                    flag_world_position = Point(x=alvo_x_global, y=alvo_y_global, z=0.0)
                    
                    self.publish_goal(flag_world_position)
                    self.state = State.MOVING_TO_FLAG
                    self.flag_pixel_pos = None # Reseta para a próxima detecção
                else:
                    # Gira para centralizar a bandeira (controle proporcional)
                    self.move_robot(0.0, 0.005 * error)

        elif self.state == State.MOVING_TO_FLAG:
            self.move_robot(0.0, 0.0) # Para e espera o path_callback mudar o estado
            
        elif self.state == State.FOLLOWING_PATH:
            # ... (sem alterações na lógica de seguir o caminho) ...
            if not self.current_path: return

            target_point = self.current_path[self.path_target_index]
            robot_pos = self.robot_pose.position
            
            distance_to_target = math.sqrt((target_point.x - robot_pos.x)**2 + (target_point.y - robot_pos.y)**2)
            angle_to_target = math.atan2(target_point.y - robot_pos.y, target_point.x - robot_pos.x)
            
            angle_error = angle_to_target - self.robot_orientation
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

            linear_speed = 0.3
            angular_speed = 0.8 * angle_error

            if abs(angle_error) > 0.2:
                linear_speed = 0.0

            self.move_robot(linear_speed, angular_speed)

            if distance_to_target < 0.25:
                self.path_target_index += 1
                if self.path_target_index >= len(self.current_path):
                    self.current_path = []
                    self.move_robot(0.0, 0.0)
                    # Lógica de decisão pós-caminho
                    if self.state != State.RETURNING_TO_BASE: # Se estava indo para a bandeira
                        self.get_logger().info("Chegou perto da bandeira! Mudando para ALIGNING_FOR_CAPTURE.")
                        self.state = State.ALIGNING_FOR_CAPTURE
                    else: # Se estava voltando para a base
                        self.get_logger().info("Chegou na base! Mudando para DEPOSITING_FLAG.")
                        self.state = State.DEPOSITING_FLAG


        elif self.state == State.ALIGNING_FOR_CAPTURE:
            # ✅ LÓGICA FINAL: Usa a visão para o alinhamento fino
            if self.flag_pixel_pos:
                error = self.camera_center_x - self.flag_pixel_pos.x
                # Se estiver alinhado e perto o suficiente
                if abs(error) < 10 and self.laser_data[0] < 0.4: # Tolerância de 10px e 40cm
                    self.move_robot(0.0, 0.0)
                    self.get_logger().info("Alinhamento final concluído. Capturando!")
                    self.state = State.CAPTURING_FLAG
                else:
                    # Anda para frente devagar enquanto corrige o ângulo
                    self.move_robot(0.05, 0.004 * error)
            else:
                # Se perdeu a visão da bandeira, gira um pouco para encontrá-la de novo
                self.move_robot(0.0, 0.2)

        elif self.state == State.CAPTURING_FLAG:
            # ... (sem alterações na sequência de captura) ...
            self.move_robot(0.0, 0.0)
            self.get_logger().info("Executando sequência de captura...")
            self.send_gripper_command(0.0, -0.06, 0.06) # 1. Abre a garra
            time.sleep(1.5)
            self.send_gripper_command(0.0, 0.0, 0.0)    # 2. Fecha a garra
            time.sleep(1.5)
            self.send_gripper_command(-0.8, 0.0, 0.0)   # 3. Eleva a garra
            time.sleep(1.5)
            self.get_logger().info("Bandeira capturada! Mudando para RETURNING_TO_BASE.")
            self.state = State.RETURNING_TO_BASE

        elif self.state == State.RETURNING_TO_BASE:
            self.move_robot(0.0, 0.0) 
            if self.base_pose:
                self.publish_goal(self.base_pose)
            else:
                self.get_logger().error("Não sei onde é a base para retornar!")
                self.state = State.DONE
            # O estado muda para FOLLOWING_PATH no 'path_callback'

        elif self.state == State.DEPOSITING_FLAG:
            # ... (sem alterações na sequência de depósito) ...
            self.move_robot(0.0, 0.0)
            self.get_logger().info("Executando sequência de depósito...")
            self.send_gripper_command(0.0, 0.0, 0.0)    # 1. Abaixa a garra
            time.sleep(1.5)
            self.send_gripper_command(0.0, -0.06, 0.06) # 2. Abre a garra
            time.sleep(1.5)
            self.get_logger().info("Bandeira depositada! Missão cumprida.")
            self.state = State.DONE

        elif self.state == State.DONE:
            self.move_robot(0.0, 0.0) 
            self.get_logger().info("Missão concluída. Desligando em 10 segundos.")
            time.sleep(10)
            self.timer_.cancel() 
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    state_machine_node = StateMachineNode()
    rclpy.spin(state_machine_node)
    state_machine_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()