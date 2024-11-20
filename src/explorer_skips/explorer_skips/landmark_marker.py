import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

class LandmarkMarker(Node):
    def __init__(self):
        super().__init__('landmark_marker')
        self.subscription = self.create_subscription(
            PointStamped,
            '/landmarks',
            self.landmark_callback,
            10)
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.no_of_markers = 0

    def landmark_callback(self, msg):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "landmarks"
        marker.id = self.no_of_markers  # Deberías asignar un ID único si tienes múltiples marcadores
        self.no_of_markers = ( self.no_of_markers + 1 ) % 20
        marker.type = Marker.TEXT_VIEW_FACING
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Posición del marcador
        marker.pose.position = msg.point
        marker.pose.orientation.w = 1.0

        marker.text = str(marker.id)  # Reemplaza con tu etiqueta

        # Escala del marcador
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04

        # Color del marcador
        marker.color.r = (0.05*self.no_of_markers) % 1.0
        marker.color.g = (0.05*self.no_of_markers+0.33) % 1.0
        marker.color.b = (0.05*self.no_of_markers+0.66) % 1.0
        marker.color.a = 1.0

        # self.get_logger().info(f'Mark published at {marker.pose.position.x}, {marker.pose.position.y}')

        # Publicar el marcador
        self.marker_publisher.publish(marker)

        # # Si tienes etiquetas, puedes publicar un marcador de texto
        # text_marker = Marker()
        # text_marker.header.frame_id = "map"
        # text_marker.header.stamp = self.get_clock().now().to_msg()
        # text_marker.ns = "landmark_labels"
        # text_marker.id = 1
        # text_marker.type = Marker.TEXT_VIEW_FACING
        # text_marker.action = Marker.ADD
        # text_marker.pose.position = msg.point
        # text_marker.pose.position.z += 0.5  # Levanta el texto sobre el marcador
        # text_marker.pose.orientation.w = 1.0

        # text_marker.text = "Etiqueta aquí"  # Reemplaza con tu etiqueta
        # text_marker.scale.z = 0.2
        # text_marker.color.r = 0.0
        # text_marker.color.g = 0.0
        # text_marker.color.b = 1.0
        # text_marker.color.a = 1.0

        # self.marker_publisher.publish(text_marker)

def main(args=None):
    rclpy.init(args=args)
    node = LandmarkMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()