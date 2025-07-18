#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point # Usaremos Point para publicar as coordenadas (x, y) do pixel

from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')
        
        # --- Publishers ---
        self.flag_pos_pub_ = self.create_publisher(Point, '/vision/flag_position', 10)
        self.base_pos_pub_ = self.create_publisher(Point, '/vision/base_position', 10)

        # --- Subscribers ---
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_callback, 10)

        self.bridge = CvBridge()
        self.get_logger().info('Nó de Visão iniciado. Aguardando imagens da câmera...')

    def camera_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # --- Detecção da Bandeira Azul ---
        blue_lower = np.array([100, 150, 50])
        blue_upper = np.array([140, 255, 255])
        blue_mask = cv2.inRange(hsv_frame, blue_lower, blue_upper)
        self.find_and_publish_blob(blue_mask, self.flag_pos_pub_, 'Bandeira Azul')

        # --- Detecção da Base Amarela ---
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv_frame, yellow_lower, yellow_upper)
        self.find_and_publish_blob(yellow_mask, self.base_pos_pub_, 'Base Amarela')

    def find_and_publish_blob(self, mask, publisher, object_name):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                point_msg = Point()
                point_msg.x = float(cx)
                point_msg.y = float(cy)
                point_msg.z = 0.0
                publisher.publish(point_msg)
                self.get_logger().info(f'{object_name} detectado em (x={cx}, y={cy})')

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()