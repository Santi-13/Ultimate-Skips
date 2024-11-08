#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2 as cv
import socket
from cv_bridge import CvBridge

class CameraClientNode(Node):
    def __init__(self):
        super().__init__('camera_client_node')

        # Configura la suscripción al tópico de la cámara
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10)
        
        # Configura el publicador para el tópico de la imagen procesada
        self.publisher_ = self.create_publisher(CompressedImage, '/processed_image', 10)

        # Configura el publicador para detecciones de hazmat
        self.hazmat_publisher = self.create_publisher(String, '/hazmatdetected', 10)

        # Configuración de CvBridge para conversión de imágenes
        self.bridge = CvBridge()

        # Configuración del servidor
        self.host = "192.168.137.165"  # Cambia a la IP del servidor si es necesario
        self.port = 10002
        self.get_logger().info("Camera client node initialized and ready.")

    def image_callback(self, data):
        try:
            # Convertimos el mensaje CompressedImage a formato cv2
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            
            # Codificamos la imagen en JPEG para enviarla
            encode_param = [int(cv.IMWRITE_JPEG_QUALITY), 70]
            result, img_enc = cv.imencode('.jpg', cv_image, encode_param)
            if not result:
                self.get_logger().warn("No se pudo codificar la imagen en JPEG.")
                return

            # Enviar la imagen comprimida al servidor
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.host, self.port))
                s.sendall(len(img_enc).to_bytes(4, byteorder="little"))
                s.sendall(img_enc.tobytes())

                # Recibir la cantidad de etiquetas detectadas
                data = s.recv(4)
                num_labels = int.from_bytes(data, byteorder="little")

                # Lista de etiquetas de hazmat
                hazmat_labels = ["hazmat", "chemical", "flammable", "toxic", "corrosive", "infectious-substance", "spontaneously-combustible", "inhalation-hazard", "radioactive",
                                 "explosive", "organic-peroxide", "non-flammable-gas", "dangerous", "flammable-solid", "oxygen", "poison"]

                # Recibir cada etiqueta detectada
                for _ in range(num_labels):
                    label_size = int.from_bytes(s.recv(4), byteorder="little")
                    label = s.recv(label_size).decode('utf-8')

                    # Verificar si la etiqueta es un hazmat y publicarla
                    if label.lower() in hazmat_labels:
                        hazmat_msg = String()
                        hazmat_msg.data = label
                        self.hazmat_publisher.publish(hazmat_msg)
                        self.get_logger().info(f"Hazmat detected: {label}")

                # Recibir el tamaño de la imagen procesada
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

def main(args=None):
    rclpy.init(args=args)
    camera_client_node = CameraClientNode()
    rclpy.spin(camera_client_node)
    camera_client_node.destroy_node()
    cv.destroyAllWindows()  # Cierra la ventana al finalizar el nodo
    rclpy.shutdown()

if __name__ == '__main__':
    main()
