#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry  
import numpy as np
import cv2 as cv
import socket
from collections import deque, Counter
from cv_bridge import CvBridge
from math import sqrt

class CameraClientNode(Node):
    def __init__(self):
        super().__init__('camera_client_node')

        # Suscripción y publicador
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(CompressedImage, '/processed_image', 10)

        # Suscripción para obtener la posición del robot
        self.position_subscription = self.create_subscription(
            Odometry,
            '/odom',  # Asumiendo que el tópico de Odometry es '/odom'
            self.position_callback,
            10)

        # Conversión de imágenes
        self.bridge = CvBridge()

        # Configuración del servidor
        self.host = "192.168.137.236"  # Cambia a la IP del servidor si es necesario
        self.port = 10002
        self.get_logger().info("Camera client node initialized and ready.")

        # Configuración de la deque para filtrado de etiquetas
        self.hazmat_queue = deque(maxlen=20)  # almacena hasta 20 etiquetas recientes
        self.hazmat_labels = set(["hazmat", "chemical", "flammable", "toxic", "corrosive",
                                  "infectious-substance", "spontaneously-combustible", "inhalation-hazard", 
                                  "radioactive", "explosive", "organic-peroxide", "non-flammable-gas",
                                  "dangerous", "flammable-solid", "oxygen", "poison"])

        # Variable para almacenar la última posición conocida
        self.current_position = None
        self.saved_positions = []  # Lista para almacenar posiciones guardadas

    def position_callback(self, msg):
        """Obtiene y guarda la posición actual del robot."""
        # Obtiene la posición (x, y) y la orientación (theta) desde el mensaje de odometría
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # La orientación está representada como un quaternion, por lo que la convertimos a ángulo (theta)
        orientation_q = msg.pose.pose.orientation
        _, _, theta = self.quaternion_to_euler(orientation_q)
        self.current_position = (x, y, theta)  # Guarda la posición y orientación actual

    def quaternion_to_euler(self, orientation_q):
        """Convierte un quaternion a ángulos de Euler (roll, pitch, yaw)."""
        import math
        x = orientation_q.x
        y = orientation_q.y
        z = orientation_q.z
        w = orientation_q.w

        # Cálculos para convertir quaternion a ángulos de Euler
        roll_x = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch_y = math.asin(2.0 * (w * y - z * x))
        yaw_z = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        return roll_x, pitch_y, yaw_z

    def image_callback(self, data):
        try:
            # Procesa la imagen recibida
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            encode_param = [int(cv.IMWRITE_JPEG_QUALITY), 70]
            result, img_enc = cv.imencode('.jpg', cv_image, encode_param)
            if not result:
                self.get_logger().warn("No se pudo codificar la imagen en JPEG.")
                return

            # Envía la imagen al servidor y recibe etiquetas detectadas
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.host, self.port))
                s.sendall(len(img_enc).to_bytes(4, byteorder="little"))
                s.sendall(img_enc.tobytes())
                data = s.recv(4)
                num_labels = int.from_bytes(data, byteorder="little")

                # Procesa las etiquetas recibidas
                detected_labels = []
                for _ in range(num_labels):
                    label_size = int.from_bytes(s.recv(4), byteorder="little")
                    label = s.recv(label_size).decode('utf-8').lower()
                    if label in self.hazmat_labels:
                        detected_labels.append(label)

                # Filtra y guarda etiquetas confiables
                for label in detected_labels:
                    self.hazmat_queue.append(label)
                    label_count = Counter(self.hazmat_queue)

                    # Verifica si la mayoría de las detecciones recientes son del mismo tipo
                    if label_count[label] >= 15:  # Por ejemplo, al menos 15 de 20
                        # Si se detecta una etiqueta hazmat confiable, guarda la posición y etiqueta en el archivo
                        if self.current_position:
                            x, y, theta = self.current_position

                            # Filtro para evitar guardar el mismo tipo de hazmat cerca de la misma posición
                            is_near_existing = False
                            for saved_position, saved_label in self.saved_positions:
                                distance = self.calculate_distance((x, y), saved_position)
                                if distance < 1.0 and saved_label == label:  # Umbral de 1 metro
                                    is_near_existing = True
                                    break

                            if not is_near_existing:
                                # Si no está cerca de una posición guardada con el mismo tipo de hazmat, lo guarda
                                with open("hazmatDetected.txt", "a") as file:
                                    file.write(f"Hazmat detected: {label}, Position: x={x}, y={y}, theta={theta}\n")
                                self.saved_positions.append(((x, y), label))  # Guarda la posición y etiqueta
                                self.get_logger().info(f"Hazmat reliably detected and recorded: {label} at position (x={x}, y={y}, theta={theta})")

                        # Limpia la queue para evitar múltiples guardados de la misma etiqueta
                        self.hazmat_queue.clear()
                        break  # Solo agrega una detección confiable a la vez

                # Recibe el tamaño de la imagen procesada
                data = s.recv(4)
                if len(data) < 4:
                    self.get_logger().warn("No se recibió el tamaño de la imagen procesada.")
                    return

                img_size = int.from_bytes(data, byteorder="little")
                b_imagen = bytearray()
                while len(b_imagen) < img_size:
                    packet = s.recv(4096)
                    if not packet:
                        break
                    b_imagen.extend(packet)

            # Verificar si la imagen completa fue recibida
            if len(b_imagen) == img_size:
                # Convertir el buffer a imagen cv2
                img_enc = np.frombuffer(b_imagen, dtype='uint8')
                processed_image = cv.imdecode(img_enc, cv.IMREAD_COLOR)

                # Mostrar la imagen procesada en una ventana
                cv.imshow("Processed Camera Feed", processed_image)
                cv.waitKey(1)  # Refresca la imagen en la ventana

                # Publicar la imagen procesada en el nuevo tópico
                msg = self.bridge.cv2_to_compressed_imgmsg(processed_image)
                self.publisher_.publish(msg)
            else:
                self.get_logger().warn("El tamaño del buffer recibido no coincide con el tamaño esperado.")

        except Exception as e:
            self.get_logger().error(f"Error en el procesamiento de la imagen: {e}")

    def calculate_distance(self, pos1, pos2):
        """Calcula la distancia euclidiana entre dos posiciones (x, y)."""
        return sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

def main(args=None):
    rclpy.init(args=args)
    camera_client_node = CameraClientNode()
    rclpy.spin(camera_client_node)
    camera_client_node.destroy_node()
    cv.destroyAllWindows()  # Cierra la ventana al finalizar el nodo
    rclpy.shutdown()

if __name__ == '__main__':
    main()
