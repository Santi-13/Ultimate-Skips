import rclpy
from rclpy.node import Node
import random
import math
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

TOTAL_HAZMATS = 4  # Número total de hazmats a detectar
RADIUS = 1.0  # Radio de distancia mínima entre puntos visitados
HAZMAT_FILE = 'hazmatDetected.txt'  # Archivo donde se almacenan los hazmats detectados
RAYCAST_RADIUS = 1.0  # Radius for raycasting

class RandomExploration(Node):
    def __init__(self):
        super().__init__('random_exploration')
        self.hazmats_detected = 0
        self.visited = []
        self.occupancy_grid = None
        self.covered_data = None  # Initialize covered_data

        # Suscribirse al mapa de ocupación
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # Suscribirse a la odometría
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Publisher to send navigation goals
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Timer para ejecutar el loop principal
        self.timer = self.create_timer(0.1, self.explore)

        # Variables para almacenar la posición y orientación actuales
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Variables para el objetivo actual
        self.goal_x = None
        self.goal_y = None

        # Flag to track if a goal is active
        self.goal_active = False

    def map_callback(self, msg):
        # Actualizar el mapa de ocupación
        self.occupancy_grid = msg
        self.get_logger().info('Occupancy grid received!')

        # Inicializar covered_data como una matriz 2D de ceros
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        self.covered_data = [[0 for _ in range(width)] for _ in range(height)]

        # Obtener límites del mapa
        self.map_min_x = self.occupancy_grid.info.origin.position.x
        self.map_min_y = self.occupancy_grid.info.origin.position.y
        self.map_max_x = self.map_min_x + self.occupancy_grid.info.width * self.occupancy_grid.info.resolution
        self.map_max_y = self.map_min_y + self.occupancy_grid.info.height * self.occupancy_grid.info.resolution


    def odom_callback(self, msg):
        # Actualizar la posición y orientación actuales
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extraer yaw del cuaternión
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def is_free(self, x, y):
        if not self.occupancy_grid:
            self.get_logger().warn('Occupancy grid not available yet.')
            return False

        map_x, map_y = self.world_to_map(x, y)
        index = map_y * self.occupancy_grid.info.width + map_x

        if 0 <= index < len(self.occupancy_grid.data):
            return self.occupancy_grid.data[index] == 0
        return False
    
    def world_to_map(self, x, y):
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        resolution = self.occupancy_grid.info.resolution
        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)
        return map_x, map_y

    def map_to_world(self, map_x, map_y):
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        resolution = self.occupancy_grid.info.resolution
        x = (map_x + 0.5) * resolution + origin_x
        y = (map_y + 0.5) * resolution + origin_y
        return x, y
    

    def is_within_radius(self, x, y):
        for xi, yi in self.visited:
            d = math.sqrt((xi - x) ** 2 + (yi - y) ** 2)
            if d < RADIUS:
                return True
        return False
    
    def is_covered(self, x, y):
        map_x, map_y = self.world_to_map(x, y)
        if 0 <= map_x < self.occupancy_grid.info.width and 0 <= map_y < self.occupancy_grid.info.height:
            return self.covered_data[map_y][map_x] == 1
        return False
    
    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y

        # Set orientation (optional, here we use the current orientation)
        q = quaternion_from_euler(0, 0, self.current_yaw)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.goal_publisher.publish(goal)
        self.get_logger().info(f'Sent new goal to ({x:.2f}, {y:.2f})')
        self.goal_active = True

    def count_hazmats_in_file(self):
        try:
            with open(HAZMAT_FILE, 'r') as file:
                lines = file.readlines()
                return len(lines)
        except FileNotFoundError:
            self.get_logger().warn(f'File {HAZMAT_FILE} not found!')
            return 0

    def explore(self):
        # Verificar si se ha recibido el mapa
        if not self.occupancy_grid:
            self.get_logger().warn('Occupancy grid not received yet.')
            return

        # Verificar si se ha recibido la odometría
        if self.current_x is None or self.current_y is None:
            self.get_logger().warn('Odometry not received yet.')
            return

        # Contar los hazmats en el archivo
        self.hazmats_detected = self.count_hazmats_in_file()

        if self.hazmats_detected >= TOTAL_HAZMATS:
            self.get_logger().info('Exploration complete!')
            self.timer.cancel()
            return

        if self.goal_active:
            # Check if we have reached the current goal
            dx = self.goal_x - self.current_x
            dy = self.goal_y - self.current_y
            distance = math.hypot(dx, dy)
            if distance < 0.2:  # Threshold to consider the goal reached
                self.get_logger().info('Reached target point!')
                self.goal_x = None
                self.goal_y = None
                self.goal_active = False
                # Perform raycasting
                self.perform_raycasting()
        else:
            # Generate a new goal
            attempts = 0
            max_attempts = 100
            while attempts < max_attempts:
                x = random.uniform(self.map_min_x, self.map_max_x)
                y = random.uniform(self.map_min_y, self.map_max_y)

                # Check if the generated position is free, not within exploration radius, and not covered
                if self.is_free(x, y) and not self.is_within_radius(x, y) and not self.is_covered(x, y):
                    self.visited.append((x, y))
                    self.goal_x = x
                    self.goal_y = y
                    self.send_goal(self.goal_x, self.goal_y)
                    break
                attempts += 1
            if attempts == max_attempts:
                self.get_logger().warn('Failed to find a valid target point.')

    def perform_raycasting(self):
        if not self.occupancy_grid:
            self.get_logger().warn('Occupancy grid not available for raycasting.')
            return

        num_rays = 360  # Número de rayos
        angle_increment = 2 * math.pi / num_rays
        for i in range(num_rays):
            angle = i * angle_increment
            self.cast_ray(angle, RAYCAST_RADIUS)

    def cast_ray(self, angle, max_distance):
        x = self.current_x
        y = self.current_y
        step_size = self.occupancy_grid.info.resolution / 2.0
        distance = 0.0

        while distance < max_distance:
            x += step_size * math.cos(angle)
            y += step_size * math.sin(angle)
            distance += step_size

            map_x, map_y = self.world_to_map(x, y)
            # Comprobar límites
            if not (0 <= map_x < self.occupancy_grid.info.width and 0 <= map_y < self.occupancy_grid.info.height):
                break

            index = map_y * self.occupancy_grid.info.width + map_x
            # Comprobar si la celda está ocupada
            if self.occupancy_grid.data[index] != 0:
                # Se encontró un obstáculo
                break

            # Marcar la celda como cubierta
            self.covered_data[map_y][map_x] = 1
            # Marcar vecinos para redundancia
            self.mark_neighbors(map_x, map_y)

    def mark_neighbors(self, map_x, map_y):
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx = map_x + dx
                ny = map_y + dy
                if 0 <= nx < self.occupancy_grid.info.width and 0 <= ny < self.occupancy_grid.info.height:
                    self.covered_data[ny][nx] = 1

    def destroy_node(self):
        # Cerrar el nodo
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    random_exploration = RandomExploration()
    rclpy.spin(random_exploration)
    random_exploration.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()