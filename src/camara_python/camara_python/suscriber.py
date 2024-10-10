#!/usr/bin/env python3

import os
os.environ["QT_QPA_PLATFORM"] = "xcb"
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        # Suscribirse al tópico '/image_raw' con mensajes de tipo Image
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.subscription  # Evita que el garbage collector elimine el suscriptor
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convertir el mensaje ROS Image a una imagen OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Mostrar la imagen
        cv2.imshow("Camera", cv_image)
        cv2.waitKey(1)  # Necesario para actualizar la ventana de OpenCV

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    # Cerrar las ventanas de OpenCV al finalizar
    cv2.destroyAllWindows()
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
