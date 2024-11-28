import rclpy
from rclpy.node import Node
import random
import math
import numpy as np
import cv2 

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped, Point, PoseWithCovarianceStamped
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler

TOTAL_HAZMATS = 4  # Número total de hazmats a detectar
RADIUS = 0.3  # Radio de distancia mínima entre puntos visitados
HAZMAT_FILE = 'hazmatDetected.txt'  # Archivo donde se almacenan los hazmats detectados
RAYCAST_RADIUS = 0.3  # Radius for raycasting
WALL_DISTANCE_THRESHOLD = RADIUS / 2.0  # Distancia mínima de los puntos a las paredes

class RandomExploration(Node):
    def __init__(self):
        super().__init__('random_exploration')
        self.hazmats_detected = 0
        self.visited = []
        self.occupancy_grid = None
        self.covered_data = None  # Initialize covered_data
        self.distance_map = None  # Initialize distance_map

        # Flags to check if map and pose are received
        self.map_received = False
        self.pose_initialized = False

        # Suscribirse al mapa de ocupación
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # Suscribirse a la odometría
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Subscribe to AMCL pose
        self.amcl_pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)

        # Publisher to send navigation goals
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Publisher to covered_data OccupancyGrid
        self.covered_map_publisher = self.create_publisher(OccupancyGrid, '/covered_map', 10)

        # Timer para ejecutar el loop principal
        self.explore_timer = self.create_timer(0.5, self.explore)

        self.landmark_pub = self.create_publisher(
            PointStamped,
            '/landmarks',
            10
        )

        # Variables para almacenar la posición y orientación actuales
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Variables para el objetivo actual
        self.goal_x = None
        self.goal_y = None

        # Flag to track if a goal is active
        self.goal_active = False

        self.goal_points = []  # List to store generated exploration points
        self.current_goal_index = 0  # Index to track the current goal

        # Timer para gestionar el retraso entre objetivos
        self.goal_timer = None

        # Offset variables to align odom with map via AMCL
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_yaw = 0.0
        self.pose_aligned = False  # Flag to ensure alignment happens only once

    def normalize_angle(self, angle):
        """
        Normalize an angle to the range [-pi, pi].
        """
        return math.atan2(math.sin(angle), math.cos(angle))

    def map_callback(self, msg):
        # Actualizar el mapa de ocupación
        self.occupancy_grid = msg
        self.get_logger().info('Occupancy grid received!')
        self.map_received = True
        
        # Publish coordinates of the dock
        dock_point = PointStamped(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='map'
            ),
            point=Point(x=9.5, y=5.0, z=0.0)
        )
        self.landmark_pub.publish(dock_point)
        self.get_logger().info('Published dock landmark.')

        # Inicializar covered_data como una matriz 2D de ceros
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        self.covered_data = [[0 for _ in range(width)] for _ in range(height)]

        # Obtener límites del mapa
        self.map_min_x = self.occupancy_grid.info.origin.position.x
        self.map_min_y = self.occupancy_grid.info.origin.position.y
        self.map_max_x = self.map_min_x + self.occupancy_grid.info.width * self.occupancy_grid.info.resolution
        self.map_max_y = self.map_min_y + self.occupancy_grid.info.height * self.occupancy_grid.info.resolution

        # Compute the distance map to the nearest wall
        self.compute_distance_map()

        # Pre-generate exploration points
        self.generate_exploration_points()

    def compute_distance_map(self):
        # Convert occupancy grid data to a 2D numpy array
        occupancy_array = np.array(self.occupancy_grid.data).reshape((self.occupancy_grid.info.height, self.occupancy_grid.info.width))
        
        # Create a binary image where occupied cells are 0 and free cells are 1
        # Assuming that any cell with value > 50 is considered occupied
        binary_image = np.where(occupancy_array > 50, 0, 1).astype(np.uint8)

        # Compute distance transform        
        self.distance_map = cv2.distanceTransform(binary_image, cv2.DIST_L2, 5)
        self.distance_map *= self.occupancy_grid.info.resolution  # Convert to meters
        self.get_logger().info('Distance map computed using OpenCV.')
        

    def generate_exploration_points(self):
        self.get_logger().info('Generating exploration points...')
        num_points = 100  # Adjust based on desired coverage and map size
        attempts = 0
        max_attempts = num_points * 100  # Prevent infinite loops

        while len(self.goal_points) < num_points and attempts < max_attempts:
            x = random.uniform(self.map_min_x, self.map_max_x)
            y = random.uniform(self.map_min_y, self.map_max_y)

            # Check if the position is free, not within exploration radius, and not covered
            if self.is_valid_point(x, y):
                # Add to visited to enforce minimum distance for future points
                self.visited.append((x, y))
                self.goal_points.append((x, y))
                self.get_logger().info(f'Generated point ({x:.2f}, {y:.2f})')

                # Publish the generated point as a landmark
                landmark_point = PointStamped(
                    header=Header(
                        stamp=self.get_clock().now().to_msg(),
                        frame_id='map'
                    ),
                    point=Point(x=x, y=y, z=0.0)
                )
                self.landmark_pub.publish(landmark_point)
                self.get_logger().info(f'Published landmark at ({x:.2f}, {y:.2f})')

                # Perform raycasting from this point to mark covered areas
                self.perform_raycasting_at_point(x, y)

            attempts += 1

        if len(self.goal_points) < num_points:
            self.get_logger().warn('Could not generate the desired number of exploration points.')

        # After generating all points, publish the covered occupancy grid
        self.publish_covered_map()

    def is_valid_point(self, x, y):
        # Check if the point is free
        if not self.is_free(x, y):
            return False

        # Check if the point is at least RADIUS away from other points
        if self.is_within_radius(x, y):
            return False

        # Check if the point is at least WALL_DISTANCE_THRESHOLD away from walls
        if not self.is_far_from_walls(x, y):
            return False

        # Check if the point is not already covered
        if self.is_covered(x, y):
            return False

        return True
    
    def is_far_from_walls(self, x, y):
        if self.distance_map is None:
            self.get_logger().warn('Distance map not available.')
            return False

        # Convert world coordinates to map indices
        map_x, map_y = self.world_to_map(x, y)

        if not (0 <= map_x < self.occupancy_grid.info.width and 0 <= map_y < self.occupancy_grid.info.height):
            return False

        # Get the distance at this cell
        distance = self.distance_map[map_y, map_x]

        if distance >= WALL_DISTANCE_THRESHOLD:
            return True
        else:
            return False
    
    def perform_raycasting_at_point(self, x, y):
        num_rays = 360  # Number of rays
        angle_increment = 2 * math.pi / num_rays

        for i in range(num_rays):
            angle = i * angle_increment
            self.cast_ray(x, y, angle, RAYCAST_RADIUS)

    def odom_callback(self, msg):
        # Update current position and orientation from odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        odom_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Normalize odom_yaw
        odom_yaw = self.normalize_angle(odom_yaw)

        # Adjusted position using offsets
        if self.pose_aligned:
            adjusted_x = self.current_x + self.offset_x
            adjusted_y = self.current_y + self.offset_y
            # Adjusted yaw if needed (not used in distance calculation)
            adjusted_yaw = self.normalize_angle(odom_yaw + self.offset_yaw)
        else:
            adjusted_x = self.current_x
            adjusted_y = self.current_y
            adjusted_yaw = odom_yaw  # No adjustment yet

        # Debug log for current goal status
        self.get_logger().debug(f'Goal active: {self.goal_active}, '
                                f'goal_x: {self.goal_x}, goal_y: {self.goal_y}')
        self.get_logger().debug(f'Adjusted Position: ({adjusted_x:.2f}, {adjusted_y:.2f}), '
                                f'Adjusted Yaw: {adjusted_yaw:.2f}')

        # Check if there is an active goal and if the robot has reached it
        if self.goal_active and self.goal_x is not None and self.goal_y is not None:
            dx = self.goal_x - adjusted_x
            dy = self.goal_y - adjusted_y
            distance = math.hypot(dx, dy)
            self.get_logger().debug(f'Distance to the goal: {distance:.2f} meters')

            if distance < 0.2:  # Threshold to consider goal reached
                self.get_logger().info('Reached target point!')
                self.goal_active = False

                # Perform raycasting at the reached point
                self.perform_raycasting_at_point(adjusted_x, adjusted_y)

                # Increment goal index to move to the next goal
                self.current_goal_index += 1

                # Introduce a delay before sending the next goal
                delay = 2.0  # seconds
                if self.goal_timer is None:
                    self.goal_timer = self.create_timer(delay, self.send_next_goal)
                    self.get_logger().info(f'Setting timer to send next goal after {delay} seconds.')


    def send_next_goal(self):
        if self.current_goal_index < len(self.goal_points):
            x, y = self.goal_points[self.current_goal_index]
            self.send_goal(x, y)
            self.get_logger().info(f'Sent goal {self.current_goal_index + 1}/{len(self.goal_points)}: ({x:.2f}, {y:.2f})')
        else:
            self.get_logger().info('All exploration points have been navigated.')
            self.timer.cancel()
        # Destruir el temporizador después de que se haya llamado
        if self.goal_timer:
            self.goal_timer.cancel()
            self.goal_timer = None

    def amcl_pose_callback(self, msg):
        # Update current position and orientation from AMCL pose
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        amcl_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Check if the pose is initialized by examining the covariance
        covariance = msg.pose.covariance
        # Simple check: if the covariance values are not all zeros, assume initialized
        if not self.pose_initialized:
            initialized = any(cov > 0.0 for cov in covariance)
            if initialized:
                self.pose_initialized = True
                self.get_logger().info('Robot pose initialized via AMCL.')

                 # Compute offsets between AMCL pose and current odom pose
                self.offset_x = msg.pose.pose.position.x - self.current_x
                self.offset_y = msg.pose.pose.position.y - self.current_y
                angle_diff = self.normalize_angle(amcl_yaw - self.current_yaw)
                self.offset_yaw = angle_diff

                self.pose_aligned = True  # Set flag to indicate alignment is done
                self.get_logger().info(f'Pose aligned with offset_x: {self.offset_x:.2f}, '
                                       f'offset_y: {self.offset_y:.2f}, offset_yaw: {self.offset_yaw:.2f}')


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

        # Establecer orientación (opcional, aquí usamos la orientación actual)
        q = quaternion_from_euler(0, 0, self.current_yaw)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        # Set the current goal coordinates for odom_callback to use
        self.goal_x = x
        self.goal_y = y

        self.goal_publisher.publish(goal)
        self.get_logger().info(f'Sent new goal to ({x:.2f}, {y:.2f})')
        self.goal_active = True

        # Publicar el punto generado como un landmark
        landmark_point = PointStamped(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id='map'
            ),
            point=Point(x=x, y=y, z=0.0)
        )
        self.landmark_pub.publish(landmark_point)
        self.get_logger().info(f'Published landmark at ({x:.2f}, {y:.2f})')


    def count_hazmats_in_file(self):
        try:
            with open(HAZMAT_FILE, 'r') as file:
                lines = file.readlines()
                return len(lines)
        except FileNotFoundError:
            self.get_logger().warn(f'File {HAZMAT_FILE} not found!')
            return 0

    def explore(self):
        # Ensure both map and pose have been received and initialized
        if not self.map_received:
            self.get_logger().warn('Waiting for the occupancy grid map...')
            return

        if not self.pose_initialized:
            self.get_logger().warn('Waiting for robot pose to be initialized via AMCL...')
            return

        # Count hazmats in the file
        self.hazmats_detected = self.count_hazmats_in_file()

        if self.hazmats_detected >= TOTAL_HAZMATS:
            self.get_logger().info('Exploration complete!')
            self.explore_timer.cancel()
            return

        # If no goal is active and there are goals left, send the next goal
        if not self.goal_active and self.current_goal_index < len(self.goal_points):
            self.send_next_goal()


    def cast_ray(self, start_x, start_y, angle, max_distance):
        x = start_x
        y = start_y
        step_size = self.occupancy_grid.info.resolution / 2.0
        distance = 0.0

        while distance < max_distance:
            x += step_size * math.cos(angle)
            y += step_size * math.sin(angle)
            distance += step_size

            map_x, map_y = self.world_to_map(x, y)
            # Check bounds
            if not (0 <= map_x < self.occupancy_grid.info.width and 0 <= map_y < self.occupancy_grid.info.height):
                break

            index = map_y * self.occupancy_grid.info.width + map_x
            # Check if the cell is occupied
            if self.occupancy_grid.data[index] != 0:
                # Obstacle encountered
                break

            # Mark the cell as covered
            self.covered_data[map_y][map_x] = 1
            # Mark neighbors for redundancy
            self.mark_neighbors(map_x, map_y)

    def mark_neighbors(self, map_x, map_y):
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx = map_x + dx
                ny = map_y + dy
                if 0 <= nx < self.occupancy_grid.info.width and 0 <= ny < self.occupancy_grid.info.height:
                    self.covered_data[ny][nx] = 1


    def publish_covered_map(self):
        if not self.covered_data or not self.occupancy_grid:
            self.get_logger().warn('Cannot publish covered map: Data not available.')
            return

        covered_map = OccupancyGrid()
        covered_map.header.frame_id = 'map'
        covered_map.header.stamp = self.get_clock().now().to_msg()
        covered_map.info = self.occupancy_grid.info  # Use the same map info as the main map

        # Flatten the 2D covered_data into a 1D list
        flat_data = []
        for row in self.covered_data:
            for cell in row:
                if cell == 1:
                    flat_data.append(50)  # Arbitrary value to represent covered areas
                else:
                    flat_data.append(-1)  # -1 represents unknown

        covered_map.data = flat_data

        # Publish the covered occupancy grid
        self.covered_map_publisher.publish(covered_map)
        self.get_logger().info('Published covered occupancy grid.')

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