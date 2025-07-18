#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class GripperTestNode(Node):
    """
    Nó de teste para enviar comandos diretos para o controlador da garra.
    """
    def __init__(self):
        super().__init__('gripper_test_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.get_logger().info('Nó de teste da garra iniciado.')
        
        # ✅ EDIÇÃO: A sequência de teste agora é chamada por um timer.
        # Isso garante que o nó esteja totalmente inicializado antes de enviar comandos.
        self.timer = self.create_timer(1.0, self.run_test_sequence)

    def send_command(self, elevation: float, left_arm_cm: float, right_arm_cm: float):
        """Envia um comando para a garra."""
        msg = Float64MultiArray()
        msg.data = [elevation, left_arm_cm, right_arm_cm]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Enviando comando: Haste={elevation:.2f} rad, Esq={left_arm_cm:.2f} cm, Dir={right_arm_cm:.2f} cm')

    def run_test_sequence(self):
        """Executa a sequência de movimentos de teste e depois sinaliza para desligar."""
        # ✅ EDIÇÃO: Cancela o timer para que a sequência rode apenas uma vez.
        self.timer.cancel()

        self.get_logger().info("--- TESTE 1: ABRINDO A GARRA ---")
        self.send_command(0.0, -0.06, 0.06)
        time.sleep(3)

        self.get_logger().info("--- TESTE 2: FECHANDO A GARRA ---")
        self.send_command(0.0, 0.0, 0.0)
        time.sleep(3)
        
        self.get_logger().info("--- TESTE 3: ELEVANDO A HASTE ---")
        self.send_command(-0.8, 0.0, 0.0)
        time.sleep(3)
        
        self.get_logger().info("--- TESTE 4: ABAIXANDO A HASTE ---")
        self.send_command(0.0, 0.0, 0.0)
        time.sleep(3)

        self.get_logger().info("--- Sequência de teste concluída. Desligando o nó. ---")
        # ✅ EDIÇÃO: A forma correta de pedir o desligamento é através do rclpy.
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GripperTestNode()
    # ✅ EDIÇÃO: A forma padrão e segura de rodar um nó.
    # O rclpy.spin irá manter o nó vivo até que rclpy.shutdown() seja chamado.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A limpeza é feita automaticamente pelo contexto do rclpy.
        node.get_logger().info('Nó finalizado.')

if __name__ == '__main__':
    main()