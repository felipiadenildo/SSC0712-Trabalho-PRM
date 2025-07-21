# -*- coding: utf-8 -*-

# Nome do Arquivo: vision_node.py
#
# Descrição:
# Este nó ROS2 é responsável pela percepção visual da missão. Ele processa o
# stream de vídeo da câmera do robô para detectar um objeto de cor específica (a "bandeira").
# Uma vez detectada, ele calcula a posição horizontal do objeto na imagem e
# publica essa informação para que outros nós, como a máquina de estados, possam
# usá-la para tarefas de alinhamento e aproximação.

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from typing import Optional

# Importações de Mensagens e Ferramentas ROS
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class VisionNode(Node):
    """
    Nó de visão que detecta a bandeira na imagem da câmera, filtra por cor e forma,
    e publica a posição horizontal (eixo X) do seu centro.
    """
    def __init__(self):
        """Construtor da classe VisionNode."""
        super().__init__('vision_node')

        # --- Parâmetros ---
        self._declare_parameters()
        self._load_parameters()

        # --- Ponte ROS <-> OpenCV ---
        # CvBridge é a ferramenta padrão para converter entre os formatos de imagem do ROS e do OpenCV.
        self.bridge = CvBridge()

        # --- QoS (Qualidade de Serviço) ---
        # Perfil para sensores: pode perder alguns pacotes, mas prioriza receber os mais recentes.
        qos_sensor_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Subscriber e Publisher ---
        self.image_subscriber = self.create_subscription(
            Image, self.p_image_topic, self.image_callback, qos_sensor_profile)
        self.flag_pos_publisher = self.create_publisher(
            Float32, self.p_detection_topic, 10)

        self.get_logger().info("✅ Vision Node iniciado. Aguardando imagens...")
        if self.p_show_debug_view:
            self.get_logger().info("   - A visualização de debug está ATIVADA.")

    def _declare_parameters(self):
        """Declara todos os parâmetros ROS para este nó."""
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detection_topic', '/camera/flag_detection')
        self.declare_parameter('show_debug_view', True)
        self.declare_parameter('min_contour_area', 300.0)

        # Parâmetros de Cor (HSV) - Faixa 1 para o Vermelho
        self.declare_parameter('hsv.h_min1', 0)
        self.declare_parameter('hsv.s_min1', 120)
        self.declare_parameter('hsv.v_min1', 70)
        self.declare_parameter('hsv.h_max1', 10)
        self.declare_parameter('hsv.s_max1', 255)
        self.declare_parameter('hsv.v_max1', 255)

        # Parâmetros de Cor (HSV) - Faixa 2 para o Vermelho (devido ao wrap-around do HUE)
        self.declare_parameter('hsv.h_min2', 170)
        self.declare_parameter('hsv.s_min2', 120)
        self.declare_parameter('hsv.v_min2', 70)
        self.declare_parameter('hsv.h_max2', 180)
        self.declare_parameter('hsv.s_max2', 255)
        self.declare_parameter('hsv.v_max2', 255)

    def _load_parameters(self):
        """Carrega os parâmetros ROS para variáveis de instância."""
        self.p_image_topic = self.get_parameter('image_topic').value
        self.p_detection_topic = self.get_parameter('detection_topic').value
        self.p_show_debug_view = self.get_parameter('show_debug_view').value
        self.p_min_area = self.get_parameter('min_contour_area').value

        # Carrega os limiares de cor em arrays numpy
        self.lower_red1 = np.array([self.get_parameter('hsv.h_min1').value, self.get_parameter('hsv.s_min1').value, self.get_parameter('hsv.v_min1').value])
        self.upper_red1 = np.array([self.get_parameter('hsv.h_max1').value, self.get_parameter('hsv.s_max1').value, self.get_parameter('hsv.v_max1').value])
        self.lower_red2 = np.array([self.get_parameter('hsv.h_min2').value, self.get_parameter('hsv.s_min2').value, self.get_parameter('hsv.v_min2').value])
        self.upper_red2 = np.array([self.get_parameter('hsv.h_max2').value, self.get_parameter('hsv.s_max2').value, self.get_parameter('hsv.v_max2').value])

    def image_callback(self, msg: Image):
        """Callback principal: processa cada quadro de imagem recebido."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Erro ao converter imagem com CvBridge: {e}")
            return

        # Cria a máscara de cor para isolar os pixels vermelhos
        mask = self._create_color_mask(cv_image)

        # Encontra e processa os contornos na máscara
        centroid_x = self._find_and_process_contours(mask)

        # Se um centroide válido foi encontrado, publica a sua posição X
        if centroid_x is not None:
            pos_msg = Float32()
            pos_msg.data = float(centroid_x)
            self.flag_pos_publisher.publish(pos_msg)

        # Se a visualização de debug estiver ativada, mostra as janelas
        if self.p_show_debug_view:
            self._show_debug_windows(cv_image, mask, centroid_x)

    def _create_color_mask(self, image: np.ndarray) -> np.ndarray:
        """
        Converte a imagem para HSV e cria uma máscara binária para a cor alvo.

        Args:
            image (np.ndarray): A imagem original no formato BGR.

        Returns:
            np.ndarray: Uma máscara binária (preto e branco) onde pixels brancos
                        correspondem à cor detectada.
        """
        # O espaço de cores HSV (Hue, Saturation, Value) é ideal para detecção de cores,
        # pois separa a intensidade da luz (V) da informação de cor (H e S).
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Como o vermelho "dá a volta" no círculo de HUE (0-180 no OpenCV),
        # precisamos de duas máscaras para capturar toda a sua faixa.
        mask1 = cv2.inRange(hsv_image, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv_image, self.lower_red2, self.upper_red2)
        
        # Combinamos as duas máscaras em uma só.
        full_mask = mask1 + mask2
        return full_mask

    def _find_and_process_contours(self, mask: np.ndarray) -> Optional[int]:
        """
        Encontra todos os contornos na máscara, filtra pelo maior e calcula seu centroide.

        Args:
            mask (np.ndarray): A máscara de cor binária.

        Returns:
            Optional[int]: A coordenada X do centroide do maior contorno válido, ou None.
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Filtra pelo maior contorno, uma heurística eficaz para ignorar ruído.
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        # Ignora contornos que são muito pequenos para serem a bandeira.
        if area < self.p_min_area:
            return None

        # "Momentos" de imagem são usados para calcular propriedades da forma, como o centroide.
        M = cv2.moments(largest_contour)
        
        # A verificação M["m00"] != 0 é crucial para evitar divisão por zero.
        if M["m00"] > 0:
            # cx = M10 / M00
            cx = int(M["m10"] / M["m00"])
            return cx
            
        return None
        
    def _show_debug_windows(self, original_image: np.ndarray, mask: np.ndarray, centroid_x: Optional[int]):
        """Mostra janelas de debug com a visão da câmera e a máscara."""
        debug_image = original_image.copy()
        
        if centroid_x is not None:
            # Desenha um círculo verde no centroide detectado para fácil visualização.
            # (Calculamos cy aqui apenas para o desenho)
            # Nota: Isso recalcula momentos, idealmente seria passado como argumento.
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > self.p_min_area:
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(debug_image, (centroid_x, cy), 7, (0, 255, 0), -1)
                        cv2.putText(debug_image, f"X: {centroid_x}", (centroid_x + 10, cy), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        cv2.imshow("Mascara de Cor", mask)
        cv2.imshow("Visao do Robo", debug_image)
        cv2.waitKey(1) # Essencial para que as janelas do OpenCV se atualizem.

def main(args=None):
    """Função principal que inicializa e executa o nó."""
    rclpy.init(args=args)
    vision_node = VisionNode()
    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        vision_node.destroy_node()
        cv2.destroyAllWindows() # Garante que as janelas do OpenCV sejam fechadas
        rclpy.shutdown()

if __name__ == '__main__':
    main()