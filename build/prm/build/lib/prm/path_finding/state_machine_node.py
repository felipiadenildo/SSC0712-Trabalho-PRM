#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from .state_machine import State
import tf_transformations
import math

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        # --- Máquina de Estados ---
        self.state = State.SEARCHING_FLAG

        # --- Sensores e Odometry ---
        self.robot_pose = None       # geometry_msgs/Pose
        self.robot_yaw = None        # float em radianos
        self.base_pose = None
        self.laser_data = None

        # --- Pure Pursuit ---
        self.current_path = []       # lista de tuples [(x, y), ...]
        self.look_ahead_dist = 0.5   # metros à frente
        self.goal_tolerance = 0.2    # quando considera missão alcançada

        # --- Publishers & Subscribers ---
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub_    = self.create_publisher(PoseStamped, '/robot/goal_pose', 10)

        self.odom_sub_ = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub_ = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.path_sub_ = self.create_subscription(
            Path, '/robot/path', self.path_callback, 10)

        # --- Timer principal ---
        self.timer_ = self.create_timer(0.1, self.execute_state_machine)

        self.twist_msg = Twist()
        self.get_logger().info('StateMachineNode iniciado.')

    def odom_callback(self, msg: Odometry):
        """Atualiza a pose e o yaw do robô."""
        pose = msg.pose.pose
        self.robot_pose = pose
        _, _, yaw = tf_transformations.euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])
        self.robot_yaw = yaw

        if self.base_pose is None:
            self.base_pose = pose
            self.get_logger().info(
                f"Base salva em x={pose.position.x:.2f}, y={pose.position.y:.2f}"
            )

    def laser_callback(self, msg: LaserScan):
        """Armazena dados do laser (poderia ser usado em avoidance)."""
        self.laser_data = msg.ranges

    def path_callback(self, msg: Path):
        """
        Recebe o Path planejado pelo A* e converte para lista de (x,y).
        Muda estado para FOLLOWING_PATH.
        """
        if self.state != State.FOLLOWING_PATH:
            pts = [pose_stamped.pose.position for pose_stamped in msg.poses]
            self.current_path = [(p.x, p.y) for p in pts]
            self.get_logger().info(
                f'Path recebido com {len(self.current_path)} waypoints. Iniciando Pure Pursuit.'
            )
            self.state = State.FOLLOWING_PATH

    def publish_goal(self, goal_point: Point):
        """Publica um PoseStamped de goal para o planejador A*."""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position = goal_point
        self.goal_pub_.publish(goal)
        self.get_logger().info(
            f"Publicado goal em x={goal_point.x:.2f}, y={goal_point.y:.2f}"
        )

    def follow_path(self):
        """
        Pure Pursuit:
         - Encontra o ponto na path que fica à look_ahead_dist do robô
         - Calcula v e w proporcionais
         - Publica Twist
        """
        if not self.current_path or self.robot_pose is None:
            return

        # Extrai posição atual
        rx, ry = self.robot_pose.position.x, self.robot_pose.position.y

        # Encontra o ponto de look‑ahead
        goal_point = None
        for (px, py) in self.current_path:
            dist = math.hypot(px - rx, py - ry)
            if dist >= self.look_ahead_dist:
                goal_point = (px, py)
                break
        # se nenhum estiver tão longe, escolhe o último (goal final)
        if goal_point is None:
            goal_point = self.current_path[-1]

        # Se já alcançou o ponto final dentro da tolerância, encerra path following
        final_dist = math.hypot(goal_point[0] - rx, goal_point[1] - ry)
        if final_dist < self.goal_tolerance:
            self.get_logger().info("Objetivo final alcançado.")
            self.current_path = []
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.cmd_vel_pub_.publish(self.twist_msg)
            self.state = State.DONE
            return

        # Calcula ângulo para o goal_point
        angle_to_goal = math.atan2(goal_point[1] - ry, goal_point[0] - rx)
        angle_error = angle_to_goal - self.robot_yaw
        # normaliza para [-π, π]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Lei de Pure Pursuit simples:
        max_lin_speed = 0.3
        kp_ang = 1.0

        # reduz v em função do erro angular (curvas mais suaves)
        v = max_lin_speed * max(0.0, 1 - abs(angle_error)/math.pi)
        w = kp_ang * angle_error

        # Preenche e publica Twist
        self.twist_msg.linear.x = v
        self.twist_msg.angular.z = w
        self.cmd_vel_pub_.publish(self.twist_msg)

    def execute_state_machine(self):
        """
        Chamado a cada 0.1s. Gerencia estados e aciona comportamento correto.
        """
        # aguarda dados
        if self.robot_pose is None or self.laser_data is None:
            return

        if self.state == State.SEARCHING_FLAG:
            # gira à procura da bandeira
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.3
            self.cmd_vel_pub_.publish(self.twist_msg)
            # simula descoberta após 5s
            if self.get_clock().now().nanoseconds / 1e9 > 5.0:
                self.state = State.MOVING_TO_FLAG

        elif self.state == State.MOVING_TO_FLAG:
            # publica goal fixo (deverá vir da visão)
            flag_pt = Point(x=5.0, y=0.0, z=0.0)
            self.publish_goal(flag_pt)
            # espera o path_callback
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.cmd_vel_pub_.publish(self.twist_msg)

        elif self.state == State.FOLLOWING_PATH:
            # aqui ocorre o Pure Pursuit
            self.follow_path()

        elif self.state == State.DONE:
            # missão concluída, para o robô
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.cmd_vel_pub_.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
