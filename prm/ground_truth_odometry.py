#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Nome do Arquivo: ground_truth_odometry.py
#
# Descrição:
# Este nó serve como uma ponte entre a informação de pose "perfeita" (ground truth)
# fornecida pelo simulador Gazebo e o ecossistema ROS.
#
# O que ele faz:
# 1. Escuta um tópico que publica a pose exata do robô no mundo (geometry_msgs/msg/Pose).
#    No nosso caso, essa informação vem da ponte Gazebo <-> ROS.
# 2. Publica essa informação em dois formatos padrão do ROS, essenciais para navegação:
#    a) Uma mensagem `nav_msgs/msg/Odometry` no tópico /odom_gt. Esta é a fonte
#       de odometria "perfeita" que a máquina de estados e outros nós usarão.
#    b) Uma transformação de coordenadas (TF) entre o frame da odometria (ex: 'odom_gt')
#       e o frame base do robô (ex: 'base_link'). Isso permite que o ROS visualize
#       e calcule posições de outros sensores em relação ao mapa.

import rclpy
from rclpy.node import Node
import numpy as np
import math
import os
import psutil

# Importações de Mensagens e Ferramentas ROS
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class GroundTruthOdomPublisher(Node):
    """
    Nó que converte uma pose 'ground truth' em uma mensagem de Odometria e uma transformação TF.
    """
    def __init__(self):
        """Construtor da classe GroundTruthOdomPublisher."""
        super().__init__('ground_truth_odom_publisher')

        # --- Parâmetros ---
        # Declarar parâmetros torna o nó reutilizável para diferentes robôs ou configurações.
        self.declare_parameter('input_pose_topic', '/odom_raw')
        self.declare_parameter('odom_topic', '/odom_gt')
        self.declare_parameter('odom_frame_id', 'odom_gt')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('use_kalman_filter', False)  # Parâmetro para ativar/desativar o filtro

        # Carrega os parâmetros para variáveis de instância
        input_topic = self.get_parameter('input_pose_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.use_kalman = self.get_parameter('use_kalman_filter').get_parameter_value().bool_value

        # --- Subscriber, Publisher e Broadcaster ---
        
        # 1. Assinatura no tópico de pose bruta vinda do simulador.
        self.create_subscription(Pose, input_topic, self.pose_callback, 10)

        # 2. Publicador da mensagem de odometria completa.
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        # 3. Broadcaster para publicar a transformação de coordenadas (TF).
        self.tf_broadcaster = TransformBroadcaster(self)

        # 4. Publicador de diagnóstico
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer_diag = self.create_timer(5.0, self.publish_diagnostics)

        # Inicialização do Filtro de Kalman
        # if self.use_kalman:
        #     self.kf = self.initialize_kalman_filter()
        #     self.last_time = self.get_clock().now()
        #     self.get_logger().info("Filtro de Kalman ativado.")
        # else:
        #     self.kf = None
        #     self.get_logger().info("Filtro de Kalman desativado.")

        self.get_logger().info(f"✅ Publicador de Odometria Ground Truth iniciado.")
        self.get_logger().info(f"   Lendo de: '{input_topic}'")
        self.get_logger().info(f"   Publicando Odometria em: '{odom_topic}'")
        self.get_logger().info(f"   Publicando TF: '{self.odom_frame}' -> '{self.base_frame}'")

    # def initialize_kalman_filter(self):
    #     """Inicializa e retorna um Filtro de Kalman para suavização da pose."""
    #     kf = KalmanFilter(dim_x=3, dim_z=3)
    #     kf.x = np.zeros(3)  # [x, y, theta]
    #     kf.F = np.eye(3)    # Matriz de transição de estado (modelo constante)
    #     kf.H = np.eye(3)    # Matriz de observação (medimos diretamente o estado)
    #     kf.P *= 1000.       # Covariância inicial (alta incerteza)
    #     kf.R = np.diag([0.1, 0.1, 0.05])  # Ruído do sensor (covariância da medição)
    #     kf.Q = np.eye(3) * 0.01  # Ruído do processo (pequeno, pois é ground truth)
    #     return kf

    def publish_diagnostics(self):
        """Publica informações de diagnóstico do nó."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = f"{self.get_name()}:Status"
        status.level = DiagnosticStatus.OK
        status.message = "Operacional"
        
        # Adicionar informações de uso de CPU
        try:
            cpu_usage = psutil.Process(os.getpid()).cpu_percent()
            status.values.append(KeyValue(key="cpu_usage", value=f"{cpu_usage:.1f}%"))
            
            if cpu_usage > 80:
                status.level = DiagnosticStatus.WARN
                status.message = "Alto uso de CPU"
        except Exception as e:
            self.get_logger().error(f"Erro ao obter uso de CPU: {str(e)}")
        
        msg.status.append(status)
        self.diagnostic_pub.publish(msg)

    def pose_callback(self, msg: Pose):
        """
        Callback executado sempre que uma nova pose 'ground truth' é recebida.
        """
        current_time = self.get_clock().now()
        now = current_time.to_msg()

        # # Converte a orientação (quaternion) para yaw (ângulo de Euler em torno do eixo Z)
        # q = msg.orientation
        # siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        # cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        # yaw = math.atan2(siny_cosp, cosy_cosp)

        # # Vetor de medição: [x, y, yaw]
        # z = np.array([msg.position.x, msg.position.y, yaw])

        # # Atualiza o Filtro de Kalman se estiver ativado
        # if self.use_kalman:
        #     # Calcula o delta de tempo desde a última atualização
        #     if hasattr(self, 'last_time'):
        #         dt = (current_time - self.last_time).nanoseconds * 1e-9
        #         # Atualiza a matriz de transição de estado (F) com o delta de tempo
        #         # Considerando um modelo de velocidade constante
        #         self.kf.F = np.array([[1, 0, 0],
        #                               [0, 1, 0],
        #                               [0, 0, 1]])
            
        #     self.kf.predict()
        #     self.kf.update(z)
        #     self.last_time = current_time

        #     # Obtém o estado filtrado
        #     x_filtered = self.kf.x
        #     x_pos = x_filtered[0]
        #     y_pos = x_filtered[1]
        #     yaw_filtered = x_filtered[2]
        # else:
        #     x_pos = msg.position.x
        #     y_pos = msg.position.y
        #     yaw_filtered = yaw

        # --- 1. Publica a mensagem de Odometria ---
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # # Define a posição filtrada
        # odom_msg.pose.pose.position.x = x_pos
        # odom_msg.pose.pose.position.y = y_pos
        # odom_msg.pose.pose.position.z = msg.position.z

        # Converte o yaw filtrado de volta para quaternion
        # odom_msg.pose.pose.orientation.z = math.sin(yaw_filtered / 2)
        # odom_msg.pose.pose.orientation.w = math.cos(yaw_filtered / 2)

        # Copia direta da pose sem filtro
        odom_msg.pose.pose = msg

        # Publica a odometria
        self.odom_pub.publish(odom_msg)

        # --- 2. Publica a Transformação de Coordenadas (TF) ---
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        
        # tf_msg.transform.translation.x = x_pos
        # tf_msg.transform.translation.y = y_pos
        # tf_msg.transform.translation.z = msg.position.z
        # tf_msg.transform.rotation.z = math.sin(yaw_filtered / 2)
        # tf_msg.transform.rotation.w = math.cos(yaw_filtered / 2)
        
        tf_msg.transform.translation.x = msg.position.x
        tf_msg.transform.translation.y = msg.position.y
        tf_msg.transform.translation.z = msg.position.z
        tf_msg.transform.rotation = msg.orientation

        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    """Função principal que inicializa e executa o nó."""
    rclpy.init(args=args)
    node = GroundTruthOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()