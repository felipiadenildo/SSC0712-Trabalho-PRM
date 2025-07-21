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

# Importações de Mensagens e Ferramentas ROS
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

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

        # Carrega os parâmetros para variáveis de instância
        input_topic = self.get_parameter('input_pose_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value

        # --- Subscriber, Publisher e Broadcaster ---
        
        # 1. Assinatura no tópico de pose bruta vinda do simulador.
        #    (No nosso launch file, remapeamos a saída do Gazebo para /odom_raw)
        self.create_subscription(Pose, input_topic, self.pose_callback, 10)

        # 2. Publicador da mensagem de odometria completa.
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        # 3. Broadcaster para publicar a transformação de coordenadas (TF).
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"✅ Publicador de Odometria Ground Truth iniciado.")
        self.get_logger().info(f"   Lendo de: '{input_topic}'")
        self.get_logger().info(f"   Publicando Odometria em: '{odom_topic}'")
        self.get_logger().info(f"   Publicando TF: '{self.odom_frame}' -> '{self.base_frame}'")

    def pose_callback(self, msg: Pose):
        """
        Callback executado sempre que uma nova pose 'ground truth' é recebida.
        """
        now = self.get_clock().now().to_msg()

        # --- 1. Publica a mensagem de Odometria ---
        # A mensagem de Odometria é mais completa que a de Pose. Ela contém um header
        # com timestamps e frames, além de campos para velocidade (que deixamos vazios).
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.odom_frame  # O frame de referência (estacionário)
        odom_msg.child_frame_id = self.base_frame # O frame que está se movendo (o robô)

        # Copia a informação de pose da mensagem de entrada para a de saída.
        odom_msg.pose.pose = msg

        # O campo 'twist' (velocidades) é deixado com zeros, pois a nossa fonte
        # de dados é apenas a pose. Para calcular a velocidade, seria necessário
        # diferenciar as poses ao longo do tempo.

        self.odom_pub.publish(odom_msg)

        # --- 2. Publica a Transformação de Coordenadas (TF) ---
        # O TF é o sistema que permite ao ROS saber onde cada componente do robô está
        # em relação a outro a qualquer momento. Publicar esta transformação é
        # essencial para que ferramentas como o RViz possam exibir o robô corretamente.
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        
        # A transformação é exatamente a pose do robô.
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