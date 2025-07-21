# prm/path_finding/vision_node.py

import rclpy
from rclpy.node import Node
import cv2  # Biblioteca OpenCV para processamento de imagem
import numpy as np
from sensor_msgs.msg import Image  # Mensagem de imagem do ROS
from std_msgs.msg import Float32   # Mensagem para publicar a posição X
from cv_bridge import CvBridge     # Ponte para converter imagens ROS <-> OpenCV

class VisionNode(Node):
    """
    Nó responsável por detectar a bandeira na imagem da câmera.
    """
    def __init__(self):
        super().__init__('vision_node')
        
        # --- Ponte ROS <-> OpenCV ---
        # CvBridge é a ferramenta padrão para converter entre os formatos de imagem
        self.bridge = CvBridge()

        # --- Subscriber ---
        # Assina o tópico de imagem da câmera do robô.
        # '/camera/image_raw' é um tópico comum, mas verifique o seu.
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.image_callback,
            10
        )

        # --- Publisher ---
        # Publica a posição X (em pixels) do centro da bandeira detectada.
        self.flag_pos_publisher = self.create_publisher(
            Float32,
            '/camera/flag_detection',
            10
        )

        # --- Definição da Cor da Bandeira (em HSV) ---
        # Estes são os valores para detectar um vermelho vivo.
        # Você PRECISARÁ ajustar estes valores para a sua simulação.
        # Limite inferior do vermelho
        self.lower_red = np.array([0, 120, 70])
        # Limite superior do vermelho
        self.upper_red = np.array([10, 255, 255])
        # Você pode precisar de uma segunda faixa para o vermelho, pois ele "dá a volta" no círculo HSV
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])
        
        self.get_logger().info("Vision Node iniciado. Aguardando imagens...")

    def image_callback(self, msg: Image):
        """
        Callback executado toda vez que uma nova imagem chega da câmera.
        """
        try:
            # 1. Converte a imagem do formato ROS para o formato OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Erro ao converter imagem: {e}")
            return

        # 2. Converte a imagem de BGR para o espaço de cores HSV
        # O espaço HSV é muito melhor para detecção de cores do que o RGB/BGR.
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 3. Cria uma "máscara" - uma imagem em preto e branco onde
        # pixels brancos representam a cor que queremos detectar.
        mask1 = cv2.inRange(hsv_image, self.lower_red, self.upper_red)
        mask2 = cv2.inRange(hsv_image, self.lower_red2, self.upper_red2)
        mask = mask1 + mask2 # Combina as duas máscaras de vermelho

        # 4. Encontra os contornos (formas) na máscara
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 5. Se algum contorno foi encontrado...
        if contours:
            # Encontra o maior contorno (provavelmente a nossa bandeira)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Calcula a área do contorno. Ignora se for muito pequeno (ruído).
            area = cv2.contourArea(largest_contour)
            if area > 300: # Limiar de área mínima (em pixels)
                # Calcula o "momento" do contorno, que nos ajuda a encontrar o centroide
                M = cv2.moments(largest_contour)
                
                # Calcula a coordenada X do centro do contorno
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    
                    # 6. Publica a posição X no tópico
                    pos_msg = Float32()
                    pos_msg.data = float(cx)
                    self.flag_pos_publisher.publish(pos_msg)
                    # Opcional: Desenha um círculo no centro para debug visual
                    # cy = int(M["m01"] / M["m00"])
                    # cv2.circle(cv_image, (cx, cy), 7, (0, 255, 0), -1)
        
        # Opcional: Mostra a imagem para debug
        # cv2.imshow("Mask", mask)
        # cv2.imshow("Vision", cv_image)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()