import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float64MultiArray
import math
from .state_machine import State
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.state = State.NAVIGATING_TO_FLAG
        self.robot_pose = None
        self.robot_orientation = None
        self.flag_pixel_pos = None
        self.laser_data = None
        self.current_path = None
        self.initial_goal_published = False

        self.sequence_start_time = None
        self.sequence_step = 0

        self.base_pose = Point(x=-2.0, y=1.0, z=0.0)
        self.flag_known_pose = Point(x=2.5, y=-2.5, z=0.0)

        self.look_ahead_dist = 0.4
        self.linear_velocity_following = 0.2
        self.angular_gain = 0.8

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/robot/goal_pose', 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        
        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)
        self.create_subscription(Float32, '/camera/flag_detection', self.flag_pos_callback, 10)
        self.create_subscription(Path, '/robot/path', self.path_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        self.timer = self.create_timer(0.1, self.execute_state_machine)
        
        self.get_logger().info(f'Máquina de Estados iniciada. Estado inicial: {self.state.name}')

        # --- PERFIL DE QUALIDADE DE SERVIÇO PARA O ALVO ---
        # Garante que a última mensagem de alvo publicada seja guardada e entregue
        # a qualquer subscritor que se conecte mais tarde.
        qos_profile_goal = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.goal_pub = self.create_publisher(PoseStamped, '/robot/goal_pose', qos_profile_goal)

    def odom_callback(self, msg: Odometry):
        self.robot_pose = msg.pose.pose
        q = msg.pose.pose.orientation
        self.robot_orientation = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def flag_pos_callback(self, msg: Float32):
        self.flag_pixel_pos = msg.data

    def laser_callback(self, msg: LaserScan):
        self.laser_data = msg.ranges

    def path_callback(self, msg: Path):
        self.get_logger().info(f'Novo caminho com {len(msg.poses)} waypoints recebido!')
        self.current_path = msg
        self.state = State.FOLLOWING_PATH
        self.get_logger().info(f"==> MUDANDO PARA O ESTADO: FOLLOWING_PATH")

    def publish_goal(self, goal_point: Point):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position = goal_point
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"==> Novo alvo publicado para o planeador: ({goal_point.x:.2f}, {goal_point.y:.2f})")

    def send_gripper_command(self, elevation: float, left_cm: float, right_cm: float):
        msg = Float64MultiArray()
        msg.data = [elevation, left_cm, right_cm]
        self.gripper_pub.publish(msg)

    def move_robot(self, linear, angular):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(twist)

    def execute_state_machine(self):
        if self.robot_pose is None:
            self.get_logger().warn('Aguardando odometria para iniciar a lógica...')
            return
        
        if self.state == State.NAVIGATING_TO_FLAG:
            if not self.initial_goal_published:
                self.get_logger().info(f"Iniciando navegação para a posição conhecida da bandeira.")
                self.publish_goal(self.flag_known_pose)
                self.initial_goal_published = True
                self.get_logger().info("Aguardando o planeador de caminho gerar a rota...")
            self.move_robot(0.0, 0.0)

        elif self.state == State.FOLLOWING_PATH:
            if self.current_path is None:
                self.move_robot(0.0, 0.0)
                return

            dist_to_goal = math.dist((self.robot_pose.position.x, self.robot_pose.position.y), 
                                      (self.current_path.poses[-1].pose.position.x, self.current_path.poses[-1].pose.position.y))

            if dist_to_goal < 0.25:
                self.move_robot(0.0, 0.0)
                final_goal = self.current_path.poses[-1].pose.position
                self.current_path = None

                is_returning = math.isclose(final_goal.x, self.base_pose.x, abs_tol=0.1)
                if is_returning:
                    self.state = State.DEPOSITING_FLAG
                    self.get_logger().info("==> Chegou na base! MUDANDO PARA DEPOSITING_FLAG")
                else:
                    self.state = State.ALIGNING_FOR_CAPTURE
                    self.get_logger().info("==> Chegou na área da bandeira! MUDANDO PARA ALIGNING_FOR_CAPTURE")
            else:
                robot_x, robot_y = self.robot_pose.position.x, self.robot_pose.position.y
                target_point = None
                for point in reversed(self.current_path.poses):
                    dist = math.dist((robot_x, robot_y), (point.pose.position.x, point.pose.position.y))
                    if dist > self.look_ahead_dist:
                        target_point = point.pose.position
                        break
                if target_point is None:
                    target_point = self.current_path.poses[-1].pose.position

                angle_to_target = math.atan2(target_point.y - robot_y, target_point.x - robot_y)
                angle_error = angle_to_target - self.robot_orientation
                angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
                angular_vel = self.angular_gain * angle_error
                self.move_robot(self.linear_velocity_following, angular_vel)

        elif self.state == State.ALIGNING_FOR_CAPTURE:
            if self.laser_data is None: return
            if self.laser_data[0] < 0.4:
                self.move_robot(0.0, 0.0)
                self.state = State.CAPTURING_FLAG
                self.get_logger().info(f"==> Perto o suficiente! MUDANDO PARA CAPTURING_FLAG")
                return
            if self.flag_pixel_pos is None:
                self.move_robot(0.0, 0.3)
                return
            error = self.flag_pixel_pos - (640.0 / 2.0)
            if abs(error) > 15:
                self.move_robot(0.0, -0.007 * error)
            else:
                self.move_robot(0.08, 0.0)

        elif self.state == State.CAPTURING_FLAG:
            if self.sequence_start_time is None:
                self.sequence_start_time = self.get_clock().now()
                self.sequence_step = 0
            now = self.get_clock().now()
            elapsed = (now - self.sequence_start_time).nanoseconds / 1e9
            if self.sequence_step == 0:
                self.send_gripper_command(0.0, -0.06, 0.06)
                self.sequence_step = 1
            elif self.sequence_step == 1 and elapsed > 1.5:
                self.send_gripper_command(0.0, 0.0, 0.0)
                self.sequence_step = 2
            elif self.sequence_step == 2 and elapsed > 3.0:
                self.send_gripper_command(-0.8, 0.0, 0.0)
                self.sequence_step = 3
            elif self.sequence_step == 3 and elapsed > 4.5:
                self.get_logger().info("### BANDEIRA CAPTURADA ###")
                self.state = State.RETURNING_TO_BASE
                self.sequence_start_time = None

        elif self.state == State.RETURNING_TO_BASE:
            self.get_logger().info("Iniciando retorno para a base...")
            self.publish_goal(self.base_pose)
            # A transição para FOLLOWING_PATH é feita no path_callback
            # Adicionamos um estado intermediário para evitar publicações repetidas
            self.state = State.SEARCHING_FLAG # Estado de espera temporário
            self.get_logger().info("Aguardando o planeador de caminho gerar a rota de retorno...")

        elif self.state == State.DEPOSITING_FLAG:
            if self.sequence_start_time is None:
                self.sequence_start_time = self.get_clock().now()
                self.sequence_step = 0
            now = self.get_clock().now()
            elapsed = (now - self.sequence_start_time).nanoseconds / 1e9
            if self.sequence_step == 0:
                self.send_gripper_command(0.0, 0.0, 0.0)
                self.sequence_step = 1
            elif self.sequence_step == 1 and elapsed > 1.5:
                self.send_gripper_command(0.0, -0.06, 0.06)
                self.sequence_step = 2
            elif self.sequence_step == 2 and elapsed > 3.0:
                self.get_logger().info("### BANDEIRA DEPOSITADA - MISSÃO CONCLUÍDA ###")
                self.state = State.DONE
                self.sequence_start_time = None

        elif self.state == State.DONE:
            self.move_robot(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    state_machine_node = StateMachineNode()
    rclpy.spin(state_machine_node)
    state_machine_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()