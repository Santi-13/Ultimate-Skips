#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import math
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist

TOTAL_HAZMATS = 4  # Número total de hazmats a detectar
RADIUS = 1.0  # Radio de distancia mínima entre puntos visitados
HAZMAT_FILE = 'hazmatDetected.txt'  # Archivo donde se almacenan los hazmats detectados


class RandomExploration(Node):
    def __init__(self):
        super().__init__('random_exploration')
        self.hazmats_detected = 0
        self.visited = []
        self.detected_positions = set()  # Para rastrear posiciones de hazmats únicos
        self.occupancy_grid = None

        # Suscribirse al mapa de ocupación
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # Suscribirse a la odometría
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Publicador para mover el robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer para ejecutar el loop principal
        self.timer = self.create_timer(0.1, self.explore)

        # Variables para almacenar la posición y orientación actuales
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Variables para el objetivo actual
        self.goal_x = None
        self.goal_y = None

    def map_callback(self, msg):
        # Actualizar el mapa de ocupación
        self.occupancy_grid = msg
        self.get_logger().info('Occupancy grid received!')

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

        width = self.occupancy_grid.info.width
        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y

        map_x = int((x - origin_x) / resolution)
        map_y = int((y - origin_y) / resolution)
        index = map_y * width + map_x

        if 0 <= index < len(self.occupancy_grid.data):
            return self.occupancy_grid.data[index] == 0
        return False

    def is_within_radius(self, x, y):
        for xi, yi in self.visited:
            d = math.sqrt((xi - x) ** 2 + (yi - y) ** 2)
            if d < RADIUS:
                return True
        return False

    def move_to(self, x, y):
        twist = Twist()

        # Calcular la diferencia
        dx = x - self.current_x
        dy = y - self.current_y

        # Distancia al objetivo
        distance = math.hypot(dx, dy)

        # Ángulo hacia el objetivo
        target_angle = math.atan2(dy, dx)

        # Diferencia de ángulo
        angle_diff = target_angle - self.current_yaw

        # Normalizar el ángulo a [-pi, pi]
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        # Constantes de control proporcional
        kp_linear = 0.5
        kp_angular = 1.0

        twist.linear.x = kp_linear * distance
        twist.angular.z = kp_angular * angle_diff

        # Limitar las velocidades
        max_linear_speed = 0.2
        max_angular_speed = 1.0
        twist.linear.x = max(min(twist.linear.x, max_linear_speed), -max_linear_speed)
        twist.angular.z = max(min(twist.angular.z, max_angular_speed), -max_angular_speed)

        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(
            f'Moving to ({x:.2f}, {y:.2f}), current position ({self.current_x:.2f}, {self.current_y:.2f}), yaw {self.current_yaw:.2f}'
        )

    def read_hazmat_file(self):
        try:
            with open(HAZMAT_FILE, 'r') as file:
                lines = file.readlines()
                for line in lines:
                    if "Hazmat detected:" in line:
                        parts = line.split("position")
                        if len(parts) > 1:
                            pos = parts[1].strip().strip("()")
                            x, y = map(float, pos.split(", "))
                            if (x, y) not in self.detected_positions:
                                self.detected_positions.add((x, y))
                                self.hazmats_detected += 1
                                self.get_logger().info(
                                    f'New hazmat detected at ({x:.2f}, {y:.2f}). Total: {self.hazmats_detected}')
        except FileNotFoundError:
            self.get_logger().warn(f'File {HAZMAT_FILE} not found!')

    def explore(self):
        # Verificar si se ha recibido el mapa
        if not self.occupancy_grid:
            self.get_logger().warn('Occupancy grid not received yet.')
            return

        # Verificar si se ha recibido la odometría
        if self.current_x is None or self.current_y is None:
            self.get_logger().warn('Odometry not received yet.')
            return

        self.read_hazmat_file()

        if self.hazmats_detected >= TOTAL_HAZMATS:
            self.get_logger().info('Exploration complete!')
            self.timer.cancel()
            # Detener el robot
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)
            return

        if self.goal_x is not None and self.goal_y is not None:
            # Verificar si hemos llegado al objetivo actual
            dx = self.goal_x - self.current_x
            dy = self.goal_y - self.current_y
            distance = math.hypot(dx, dy)
            if distance < 0.1:  # Umbral para considerar que se alcanzó el objetivo
                self.get_logger().info('Reached target point!')
                self.goal_x = None
                self.goal_y = None
                # Detener el robot
                twist = Twist()
                self.cmd_vel_publisher.publish(twist)
            else:
                # Continuar moviéndose hacia el objetivo
                self.move_to(self.goal_x, self.goal_y)
        else:
            # Generar un nuevo objetivo
            attempts = 0
            max_attempts = 100
            while attempts < max_attempts:
                x = random.uniform(self.map_min_x, self.map_max_x)
                y = random.uniform(self.map_min_y, self.map_max_y)

                # Verificar si la posición generada está libre y no está dentro del radio de exploración
                if self.is_free(x, y) and not self.is_within_radius(x, y):
                    self.visited.append((x, y))
                    self.goal_x = x
                    self.goal_y = y
                    self.get_logger().info(f'New target set to ({x:.2f}, {y:.2f})')
                    break
                attempts += 1
            if attempts == max_attempts:
                self.get_logger().warn('Failed to find a valid target point.')

    def destroy_node(self):
        # Detener el robot al cerrar el nodo
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    random_exploration = RandomExploration()
    rclpy.spin(random_exploration)
    random_exploration.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()