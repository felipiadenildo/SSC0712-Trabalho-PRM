# -*- coding: utf-8 -*-

# Nome do Arquivo: state_machine_node.py
#
# Descrição:
# Este nó é o cérebro da operação, implementando uma máquina de estados finitos (FSM)
# para gerenciar o comportamento do robô durante a missão de "Capturar a Bandeira".
# Ele coordena a navegação, percepção e atuação para cumprir os objetivos.
#
# Objetivos do Projeto Atendidos:
# - Locate the flag: Navega para uma posição pré-definida e depois usa a câmera.
# - Capture the flag: Executa uma sequência de movimentos e comandos da garra.
# - Return to base: Publica a localização da base como um alvo para o planejador.
# - Deposit the flag: Executa uma sequência para soltar a bandeira na base.
# - Avoid collision: Deleta a tarefa de evitar colisões para o planejador de caminho
#   e o seguidor de caminho (Pure Pursuit).

import rclpy
from rclpy.node import Node
import math
import os
import psutil
from datetime import datetime, timedelta

# Importações de Mensagens ROS
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float64MultiArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Importações de Qualidade de Serviço (QoS)
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from rclpy.duration import Duration

# Importa a definição dos estados
from .state_machine import State

class StateMachineNode(Node):
    """
    Nó que implementa a lógica principal de controle da missão 'Capturar a Bandeira'.
    """
    def __init__(self):
        """Construtor da classe StateMachineNode."""
        super().__init__('state_machine_node')

        # --- Declaração de Parâmetros ---
        # A declaração de parâmetros torna o nó altamente configurável sem alterar o código.
        
        # Parâmetros de Tópicos
        self.declare_parameter('odom_topic', '/odom_gt')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('path_topic', '/robot/path')
        self.declare_parameter('goal_topic', '/robot/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('gripper_topic', '/gripper_controller/commands')
        self.declare_parameter('flag_detection_topic', '/camera/flag_detection')

        # Parâmetros de Posições Chave
        self.declare_parameter('flag_known_pose.x', 2.5)
        self.declare_parameter('flag_known_pose.y', -2.5)
        self.declare_parameter('base_pose.x', -8.0)
        self.declare_parameter('base_pose.y', -0.5)

        # Parâmetros do Controlador "Pure Pursuit" (Seguidor de Caminho)
        self.declare_parameter('look_ahead_dist', 0.4)
        self.declare_parameter('path_linear_velocity', 0.9)
        self.declare_parameter('path_angular_gain', 0.8)
        self.declare_parameter('goal_tolerance', 0.25)

        # Parâmetros do Alinhamento Fino (Visão e Laser)
        self.declare_parameter('alignment_dist_laser', 0.35)
        self.declare_parameter('alignment_image_width', 640.0)
        self.declare_parameter('alignment_pixel_tolerance', 15.0)
        self.declare_parameter('alignment_angular_gain', -0.007)
        self.declare_parameter('alignment_linear_velocity', 0.08)
        self.declare_parameter('alignment_search_rotation_speed', 0.3)
        
        # --- Obtenção dos Parâmetros ---
        self._load_parameters()
        
        # --- Estado Inicial ---
        self.state = State.NAVIGATING_TO_FLAG
        
        # --- Variáveis de Instância ---
        self.robot_pose = None         # Armazena a pose completa do robô
        self.robot_yaw = None          # Armazena apenas a orientação yaw (ângulo)
        self.flag_pixel_pos = None     # Posição horizontal do pixel da bandeira na imagem
        self.laser_ranges = None       # Leituras do scanner a laser
        self.current_path = None       # Caminho atual sendo seguido
        self.goal_published = False    # Flag para controlar publicações de alvo
        self.sequence_start_time = None # Temporizador para sequências de ações
        self.sequence_step = 0          # Passo atual em uma sequência
        self.state_start_time = self.get_clock().now()  # Tempo de entrada no estado atual
        self.current_goal = None        # Armazena o objetivo atual
        self.prev_error = 0.0           # Erro anterior para controle PID
        self.integral = 0.0             # Termo integral para controle PID
        self.recovery_start_time = None # Tempo de início da recuperação

        # --- Qualidade de Serviço (QoS) ---
        # Perfil para dados que devem ser recebidos confiavelmente
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=10
        )
        
        # Perfil para "latched topics" (tópicos travados), como o alvo e o mapa.
        # Garante que a última mensagem seja guardada e entregue a qualquer nó que se conecte depois.
        qos_transient_local = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, self.p_cmd_vel_topic, qos_reliable)
        self.goal_pub = self.create_publisher(PoseStamped, self.p_goal_topic, qos_transient_local)
        self.gripper_pub = self.create_publisher(Float64MultiArray, self.p_gripper_topic, qos_reliable)
        
        # --- Subscribers ---
        self.create_subscription(Odometry, self.p_odom_topic, self.odom_callback, qos_reliable)
        self.create_subscription(Float32, self.p_flag_detection_topic, self.flag_pos_callback, qos_reliable)
        self.create_subscription(Path, self.p_path_topic, self.path_callback, qos_reliable)
        self.create_subscription(LaserScan, self.p_scan_topic, self.laser_callback, qos_reliable)

        # --- Timer Principal ---
        # O timer dispara a execução da máquina de estados a 10Hz (a cada 0.1s)
        self.timer = self.create_timer(0.1, self.execute_state_machine)
        
        # --- Timer de Diagnóstico ---
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer_diag = self.create_timer(5.0, self.publish_diagnostics)
        
        self.get_logger().info(f"✅ Máquina de Estados iniciada. Estado inicial: {self.state.name}")

    def _load_parameters(self):
        """Carrega todos os parâmetros ROS para variáveis de instância."""
        # Tópicos
        self.p_odom_topic = self.get_parameter('odom_topic').value
        self.p_scan_topic = self.get_parameter('scan_topic').value
        self.p_path_topic = self.get_parameter('path_topic').value
        self.p_goal_topic = self.get_parameter('goal_topic').value
        self.p_cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.p_gripper_topic = self.get_parameter('gripper_topic').value
        self.p_flag_detection_topic = self.get_parameter('flag_detection_topic').value
        
        # Posições
        self.flag_known_pose = Point(
            x=float(self.get_parameter('flag_known_pose.x').value),
            y=float(self.get_parameter('flag_known_pose.y').value),
            z=0.0
        )
        self.base_pose = Point(
            x=float(self.get_parameter('base_pose.x').value),
            y=float(self.get_parameter('base_pose.y').value),
            z=0.0
        )
        
        # Pure Pursuit
        self.p_look_ahead_dist = float(self.get_parameter('look_ahead_dist').value)
        self.p_linear_velocity = float(self.get_parameter('path_linear_velocity').value)
        self.p_angular_gain = float(self.get_parameter('path_angular_gain').value)
        self.p_goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        # Alinhamento
        self.p_align_dist = float(self.get_parameter('alignment_dist_laser').value)
        self.p_image_center = float(self.get_parameter('alignment_image_width').value) / 2.0
        self.p_pixel_tolerance = float(self.get_parameter('alignment_pixel_tolerance').value)
        self.p_align_ang_gain = float(self.get_parameter('alignment_angular_gain').value)
        self.p_align_lin_vel = float(self.get_parameter('alignment_linear_velocity').value)
        self.p_search_rot_vel = float(self.get_parameter('alignment_search_rotation_speed').value)
        
        self.get_logger().info("Parâmetros carregados com sucesso.")

    def publish_diagnostics(self):
        """Publica informações de diagnóstico do sistema."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Status do nó
        status = DiagnosticStatus()
        status.name = f"{self.get_name()}:Status"
        status.level = DiagnosticStatus.OK
        status.message = f"Estado atual: {self.state.name}"
        status.values.append(KeyValue(key="current_state", value=self.state.name))
        
        # Uso de CPU
        cpu_status = DiagnosticStatus()
        cpu_status.name = f"{self.get_name()}:CPU"
        try:
            cpu_usage = psutil.Process(os.getpid()).cpu_percent()
            cpu_status.values.append(KeyValue(key="cpu_usage", value=f"{cpu_usage:.1f}%"))
            
            if cpu_usage > 80:
                cpu_status.level = DiagnosticStatus.WARN
                cpu_status.message = "Alto uso de CPU"
            else:
                cpu_status.level = DiagnosticStatus.OK
                cpu_status.message = "Uso normal"
        except Exception as e:
            cpu_status.level = DiagnosticStatus.ERROR
            cpu_status.message = f"Erro ao ler CPU: {str(e)}"
        
        msg.status.append(status)
        msg.status.append(cpu_status)
        self.diagnostic_pub.publish(msg)

    # --- Funções de Callback (recebem dados dos sensores) ---

    def odom_callback(self, msg: Odometry):
        """Atualiza a posição e orientação do robô."""
        self.robot_pose = msg.pose.pose
        # Converte o quaternião de orientação para um ângulo Yaw (em radianos)
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def flag_pos_callback(self, msg: Float32):
        """Armazena a posição horizontal da bandeira detectada na imagem."""
        self.flag_pixel_pos = msg.data

    def laser_callback(self, msg: LaserScan):
        """Armazena as leituras do laser."""
        self.laser_ranges = msg.ranges

    def path_callback(self, msg: Path):
        """
        Recebe um novo caminho do planejador e muda o estado para segui-lo.
        Este é um gatilho fundamental para a máquina de estados.
        """
        if not msg.poses:
            self.get_logger().warn("Recebido um caminho vazio. Ignorando.")
            return

        self.get_logger().info(f"🗺️ Novo caminho com {len(msg.poses)} waypoints recebido!")
        self.current_path = msg
        self.change_state(State.FOLLOWING_PATH)

    # --- Funções de Ação (publicam comandos) ---

    def publish_goal(self, goal_point: Point):
        """Publica uma nova pose de destino para o planejador de caminho."""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position = goal_point
        goal_msg.pose.orientation.w = 1.0 # Orientação neutra
        self.goal_pub.publish(goal_msg)
        self.current_goal = goal_point
        self.get_logger().info(f"🎯 Novo alvo publicado para o planejador: ({goal_point.x:.2f}, {goal_point.y:.2f})")
        self.goal_published = True

    def send_gripper_command(self, elevation: float, left_cm: float, right_cm: float):
        """Envia comandos para o controlador da garra."""
        msg = Float64MultiArray()
        msg.data = [float(elevation), float(left_cm), float(right_cm)]
        self.gripper_pub.publish(msg)

    def move_robot(self, linear: float, angular: float):
        """Publica comandos de velocidade linear e angular."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

    # --- Lógica da Máquina de Estados ---
    
    def change_state(self, new_state: State):
        """Muda para um novo estado e registra a transição."""
        if self.state != new_state:
            self.get_logger().info(f"==> MUDANÇA DE ESTADO: {self.state.name} -> {new_state.name}")
            self.state = new_state
            self.state_start_time = self.get_clock().now()
            
            # Reseta variáveis de controle de estado/sequência
            self.goal_published = False
            self.sequence_start_time = None
            self.sequence_step = 0
            self.prev_error = 0.0
            self.integral = 0.0

    def execute_state_machine(self):
        """Função principal executada pelo timer, contendo a lógica de cada estado."""
        if self.robot_pose is None or self.robot_yaw is None:
            self.get_logger().warn('Aguardando odometria para iniciar a lógica...', throttle_duration_sec=5)
            return
        
        # Verificação de tempo limite para estados críticos
        if self.state in [State.FOLLOWING_PATH, State.ALIGNING_FOR_CAPTURE]:
            elapsed_time = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
            if elapsed_time > 30.0:  # 30 segundos de timeout
                self.get_logger().error(f"Timeout no estado {self.state.name}! Ativando modo de recuperação.")
                self.change_state(State.RECOVERY)
        
        # --- ESTADO 1: NAVEGANDO PARA A BANDEIRA ---
        if self.state == State.NAVIGATING_TO_FLAG:
            if not self.goal_published:
                self.get_logger().info("Iniciando navegação para a posição conhecida da bandeira.")
                self.publish_goal(self.flag_known_pose)
                self.change_state(State.WAITING_FOR_PATH) # Muda para um estado de espera

        # --- ESTADO DE ESPERA: AGUARDANDO CAMINHO ---
        elif self.state == State.WAITING_FOR_PATH:
            # Neste estado, o robô não faz nada, apenas espera o path_callback
            # ser ativado pelo planejador. Isso evita publicações repetidas de alvo.
            self.move_robot(0.0, 0.0)

        # --- ESTADO 2: SEGUINDO O CAMINHO (PURE PURSUIT) ---
        elif self.state == State.FOLLOWING_PATH:
            # Verificação de obstáculos próximos
            if self.laser_ranges:
                # Ângulo de 60° à frente (considerando 360 leituras)
                num_ranges = len(self.laser_ranges)
                front_angle = int(num_ranges * 60 / 360 / 2)
                front_cone = self.laser_ranges[:front_angle] + self.laser_ranges[-front_angle:]
                
                # Filtra leituras infinitas
                valid_ranges = [r for r in front_cone if not math.isinf(r)]
                if valid_ranges:
                    min_distance = min(valid_ranges)
                    
                    if min_distance < 0.5:  # 50cm de limite de segurança
                        self.move_robot(0.0, 0.0)
                        self.get_logger().warn(f"Obstáculo detectado a {min_distance:.2f}m! Replanejando...")
                        if self.current_goal:
                            self.publish_goal(self.current_goal)
                            self.change_state(State.WAITING_FOR_PATH)
                        return
            
            if self.current_path is None:
                self.move_robot(0.0, 0.0)
                return

            robot_pos = self.robot_pose.position
            final_goal_pos = self.current_path.poses[-1].pose.position
            dist_to_final_goal = math.dist((robot_pos.x, robot_pos.y), (final_goal_pos.x, final_goal_pos.y))

            # Verifica se chegou ao destino final do caminho
            if dist_to_final_goal < self.p_goal_tolerance:
                self.move_robot(0.0, 0.0)
                path_destination = self.current_path.poses[-1].pose.position
                self.current_path = None # Limpa o caminho atual

                # Decide para qual estado ir com base no destino que acabou de alcançar
                is_at_base = math.isclose(path_destination.x, self.base_pose.x, abs_tol=0.1) and \
                             math.isclose(path_destination.y, self.base_pose.y, abs_tol=0.1)
                
                if is_at_base:
                    self.change_state(State.DEPOSITING_FLAG)
                else:
                    self.change_state(State.ALIGNING_FOR_CAPTURE)
                return

            # Lógica do Pure Pursuit com busca binária
            target_point = self._find_target_point()
            if target_point is None: 
                target_point = final_goal_pos # Se não encontrar, mira no final

            angle_to_target = math.atan2(target_point.y - robot_pos.y, target_point.x - robot_pos.x)
            angle_error = self._normalize_angle(angle_to_target - self.robot_yaw)
            
            angular_vel = self.p_angular_gain * angle_error
            self.move_robot(self.p_linear_velocity, angular_vel)

        # --- ESTADO 3: ALINHAMENTO FINO PARA CAPTURA ---
        elif self.state == State.ALIGNING_FOR_CAPTURE:
            if self.laser_ranges is None: 
                return

            # 1. Verificação de distância com laser
            if min(self.laser_ranges[0:10]) < self.p_align_dist:
                self.move_robot(0.0, 0.0)
                self.change_state(State.CAPTURING_FLAG)
                return

            # 2. Centralização com a câmera usando controle PID
            if self.flag_pixel_pos is not None:
                error = self.flag_pixel_pos - self.p_image_center
                
                # Controle PID para alinhamento angular
                P = error * 0.01
                self.integral += error * 0.001
                D = (error - self.prev_error) * 0.005
                self.prev_error = error
                
                angular_vel = P + self.integral + D
                
                # Se o erro for pequeno, avance
                if abs(error) < self.p_pixel_tolerance:
                    linear_vel = self.p_align_lin_vel
                else:
                    linear_vel = 0.0
                    
                self.move_robot(linear_vel, angular_vel)
            else:
                # Se não detectou a bandeira, gira para procurar
                self.move_robot(0.0, self.p_search_rot_vel)

        # --- ESTADO 4: CAPTURANDO A BANDEIRA (SEQUÊNCIA TEMPORIZADA) ---
        elif self.state == State.CAPTURING_FLAG:
            self._execute_timed_sequence([
                (0.0, lambda: self.send_gripper_command(0.0, -0.06, 0.06)), # Abre a garra
                (1.5, lambda: self.send_gripper_command(0.0, 0.0, 0.0)),   # Fecha a garra
                (3.0, lambda: self.send_gripper_command(-0.8, 0.0, 0.0)),  # Eleva a garra
                (4.5, lambda: self.change_state(State.RETURNING_TO_BASE))  # Fim da sequência
            ])

        # --- ESTADO 5: INICIANDO O RETORNO PARA A BASE ---
        elif self.state == State.RETURNING_TO_BASE:
            if not self.goal_published:
                self.get_logger().info("Iniciando retorno para a base...")
                self.publish_goal(self.base_pose)
                self.change_state(State.WAITING_FOR_PATH) # Reutiliza o estado de espera

        # --- ESTADO 6: DEPOSITANDO A BANDEIRA (SEQUÊNCIA TEMPORIZADA) ---
        elif self.state == State.DEPOSITING_FLAG:
            self._execute_timed_sequence([
                (0.0, lambda: self.send_gripper_command(0.0, 0.0, 0.0)),    # Abaixa a garra
                (1.5, lambda: self.send_gripper_command(0.0, -0.06, 0.06)), # Abre a garra
                (3.0, lambda: self.change_state(State.DONE))                # Fim da missão
            ])
            
        # --- ESTADO FINAL: MISSÃO CONCLUÍDA ---
        elif self.state == State.DONE:
            self.get_logger().info("### MISSÃO CONCLUÍDA ###", once=True)
            self.move_robot(0.0, 0.0)
            
        # --- ESTADO DE RECUPERAÇÃO ---
        elif self.state == State.RECOVERY:
            if self.sequence_step == 0:
                self.get_logger().warn("Modo de recuperação: recuando...")
                self.move_robot(-0.2, 0.0)
                self.recovery_start_time = self.get_clock().now()
                self.sequence_step = 1
                
            elif self.sequence_step == 1:
                elapsed = (self.get_clock().now() - self.recovery_start_time).nanoseconds / 1e9
                if elapsed > 2.0:  # Recua por 2 segundos
                    self.get_logger().warn("Modo de recuperação: girando...")
                    self.move_robot(0.0, 0.5)
                    self.recovery_start_time = self.get_clock().now()
                    self.sequence_step = 2
                    
            elif self.sequence_step == 2:
                elapsed = (self.get_clock().now() - self.recovery_start_time).nanoseconds / 1e9
                if elapsed > 3.0:  # Gira por 3 segundos
                    self.get_logger().warn("Modo de recuperação: retomando missão...")
                    if self.current_goal:
                        self.publish_goal(self.current_goal)
                        self.change_state(State.WAITING_FOR_PATH)
                    else:
                        self.change_state(State.NAVIGATING_TO_FLAG)
            
    # --- Funções Auxiliares da Lógica ---

    def _find_target_point(self) -> Point | None:
        """Encontra o ponto alvo ideal usando busca binária."""
        if self.current_path is None or not self.current_path.poses:
            return None
            
        robot_pos = (self.robot_pose.position.x, self.robot_pose.position.y)
        low, high = 0, len(self.current_path.poses) - 1
        
        # Busca binária para encontrar segmento mais próximo
        while high - low > 1:
            mid = (low + high) // 2
            mid_point = self.current_path.poses[mid].pose.position
            dist_mid = math.dist(robot_pos, (mid_point.x, mid_point.y))
            
            if dist_mid < self.p_look_ahead_dist:
                low = mid
            else:
                high = mid
        
        # Encontrar ponto exato no segmento
        for i in range(low, min(high + 1, len(self.current_path.poses))):
            point = self.current_path.poses[i].pose.position
            dist = math.dist(robot_pos, (point.x, point.y))
            if dist >= self.p_look_ahead_dist:
                return point
        
        return self.current_path.poses[-1].pose.position

    def _execute_timed_sequence(self, steps: list[tuple[float, callable]]):
        """Executa uma lista de ações baseadas no tempo decorrido."""
        if self.sequence_start_time is None:
            self.sequence_start_time = self.get_clock().now()
            self.sequence_step = 0
            
        now = self.get_clock().now()
        elapsed = (now - self.sequence_start_time).nanoseconds / 1e9

        # Verifica se é hora de executar o próximo passo
        if self.sequence_step < len(steps) and elapsed >= steps[self.sequence_step][0]:
            action_time, action_function = steps[self.sequence_step]
            self.get_logger().info(f"Executando passo {self.sequence_step} da sequência após {elapsed:.2f}s.")
            try:
                action_function()
            except Exception as e:
                self.get_logger().error(f"Erro na sequência: {str(e)}")
            self.sequence_step += 1

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normaliza um ângulo para o intervalo [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    """Função principal que inicializa e executa o nó."""
    rclpy.init(args=args)
    state_machine_node = StateMachineNode()
    try:
        rclpy.spin(state_machine_node)
    except KeyboardInterrupt:
        pass
    finally:
        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()