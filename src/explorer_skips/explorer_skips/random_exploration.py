import rclpy
from rclpy.node import Node
import random
import math
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml
import os

class RandomExploration(Node):
    def __init__(self):
        super().__init__('random_exploration')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Cargar los límites del mapa desde map.yaml
        self.map_bounds = self.load_map_bounds('path/to/map.yaml')

        self.get_logger().info("Random exploration node initialized.")
        
        # Iniciar la exploración aleatoria si el mapa fue cargado correctamente
        if self.map_bounds:
            self.timer = self.create_timer(2.0, self.start_random_exploration)
        else:
            self.get_logger().error("Failed to load map bounds. Exploration not started.")

    def load_map_bounds(self, yaml_path):
        """Carga los límites del mapa desde el archivo map.yaml."""
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"Map YAML file not found at: {yaml_path}")
            return None

        with open(yaml_path, 'r') as file:
            map_data = yaml.safe_load(file)

        resolution = map_data['resolution']       # Tamaño de cada celda en metros
        width = map_data['image']                 # Ancho del mapa en píxeles
        height = map_data['image']                # Alto del mapa en píxeles
        origin = map_data['origin']               # Coordenadas de origen (x, y)

        # Calcular los límites del mapa
        x_min = origin[0]
        y_min = origin[1]
        x_max = x_min + (width * resolution)
        y_max = y_min + (height * resolution)

        map_bounds = {
            'x_min': x_min,
            'x_max': x_max,
            'y_min': y_min,
            'y_max': y_max
        }
        self.get_logger().info(f"Map bounds set: {map_bounds}")
        return map_bounds

    def start_random_exploration(self):
        """Inicia el ciclo de exploración aleatoria una vez se establecen los límites del mapa."""
        if self.map_bounds is None:
            self.get_logger().info("Map bounds not set yet, waiting for map data.")
            return  # Esperar hasta que se carguen los límites del mapa

        self.get_logger().info("Starting random exploration.")
        target_x = random.uniform(self.map_bounds['x_min'], self.map_bounds['x_max'])
        target_y = random.uniform(self.map_bounds['y_min'], self.map_bounds['y_max'])
        self.get_logger().info(f"Generated random target point: x={target_x}, y={target_y}")
        
        if self.navigate_to_point(target_x, target_y):
            self.get_logger().info("Reached target point. Performing 360-degree scan.")
            self.rotate_360()

    def navigate_to_point(self, x, y):
        """Envía una acción de navegación al robot para ir a la posición (x, y)."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        # Esperar a que el cliente esté disponible
        self.nav_to_pose_client.wait_for_server()
        
        self.get_logger().info("Sending navigation goal.")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected!")
            return False
        
        self.get_logger().info("Goal accepted, waiting for result.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("Reached the target point.")
            return True
        else:
            self.get_logger().warn("Failed to reach the target point.")
            return False

    def rotate_360(self):
        """Gira el robot 360 grados en el lugar."""
        twist_msg = Twist()
        twist_msg.angular.z = 1.0

        rotation_duration = 2 * math.pi / twist_msg.angular.z
        end_time = self.get_clock().now() + rclpy.time.Duration(seconds=rotation_duration)

        while self.get_clock().now() < end_time:
            self.cmd_pub.publish(twist_msg)

        twist_msg.angular.z = 0.0
        self.cmd_pub.publish(twist_msg)
        self.get_logger().info("Completed 360-degree rotation.")

def main(args=None):
    rclpy.init(args=args)
    node = RandomExploration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Random exploration interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
