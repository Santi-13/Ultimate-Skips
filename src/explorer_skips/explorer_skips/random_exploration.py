#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import math
from nav_msgs.msg import OccupancyGrid
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

        # Publicador para mover el robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer para ejecutar el loop principal
        self.timer = self.create_timer(1.0, self.explore)

    def map_callback(self, msg):
        # Actualizar el mapa de ocupación
        self.occupancy_grid = msg
        self.get_logger().info('Occupancy grid received!')

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
            d = math.sqrt((xi - x)**2 + (yi - y)**2)
            if d < RADIUS:
                return True
        return False

    def move_to(self, x, y):
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Moving to ({x:.2f}, {y:.2f})')

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
        self.read_hazmat_file()

        if self.hazmats_detected >= TOTAL_HAZMATS:
            self.get_logger().info('Exploration complete!')
            self.timer.cancel()
            return

        x = random.uniform(-10.0, 10.0)
        y = random.uniform(-10.0, 10.0)

        if self.is_free(x, y) and not self.is_within_radius(x, y):
            self.visited.append((x, y))
            self.move_to(x, y)


def main(args=None):
    rclpy.init(args=args)
    random_exploration = RandomExploration()
    rclpy.spin(random_exploration)
    random_exploration.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
