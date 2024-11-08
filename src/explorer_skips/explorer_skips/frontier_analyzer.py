import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point
import numpy as np
import math

class FrontierAnalyzer(Node):
    def __init__(self):
        super().__init__('frontier_analyzer')
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        self.pose_sub = self.create_subscription(
            Odometry,
            'odom',
            self.pose_callback,
            10
        )
        self.goal_pub = self.create_publisher(
            Point,
            'unknown_frontier_goal',
            10
        )
        self.goal_active = False
        self.current_position = None
        self.goal_point = None

    def map_callback(self, msg):
        # Process the OccupancyGrid to find frontiers
        frontiers = self.find_frontiers(msg)
        if frontiers:
            # Choose a frontier point (e.g., the closest one)
            goal_point = self.select_goal(frontiers)
            # Publish the goal point
            self.goal_pub.publish(goal_point)
            self.goal_point = goal_point
            self.goal_active = True
            self.get_logger().info(f'Published goal point: x={goal_point.x}, y={goal_point.y}')
        else:
            self.get_logger().info('No frontiers found.')

    def pose_callback(self, msg):
        # self.get_logger().info('Pose Callback')
        self.current_position = msg.pose.pose.position
        if self.goal_active and self.goal_point is not None:
            distance = math.hypot(
                self.goal_point.x - self.current_position.x,
                self.goal_point.y - self.current_position.y
            )
            if distance < 0.5:  # Goal tolerance in meters
                # self.get_logger().info('Goal reached.')
                self.goal_active = False
                self.goal_point = None

    def find_frontiers(self, occupancy_grid):
        # Convert the 1D data array into a 2D numpy array
        data = np.array(occupancy_grid.data).reshape(
            (occupancy_grid.info.height, occupancy_grid.info.width)
        )
        frontiers = []
        resolution = occupancy_grid.info.resolution
        origin_x = occupancy_grid.info.origin.position.x
        origin_y = occupancy_grid.info.origin.position.y

        

        # Iterate over each cell in the grid
        for i in range(occupancy_grid.info.height):
            for j in range(occupancy_grid.info.width):
                if data[i][j] == 0:
                    # Check 4-connected neighbors
                    neighbors = self.get_neighbors(
                        i, j, occupancy_grid.info.height, occupancy_grid.info.width
                    )
                    for n in neighbors:
                        ni, nj = n
                        if data[ni][nj] == -1:
                            # Frontier found
                            # Convert grid index to world coordinates
                            x = origin_x + (j + 0.5) * resolution
                            y = origin_y + (i + 0.5) * resolution
                            point = Point()
                            point.x = x
                            point.y = y
                            point.z = 0.0
                            frontiers.append(point)
                            self.get_logger().info(f"New point detected at {point}")
                            break  # No need to check other neighbors
        return frontiers

    def get_neighbors(self, i, j, height, width):
        neighbors = []
        if i > 0:
            neighbors.append((i - 1, j))
        if i < height - 1:
            neighbors.append((i + 1, j))
        if j > 0:
            neighbors.append((i, j - 1))
        if j < width - 1:
            neighbors.append((i, j + 1))
        return neighbors

    def select_goal(self, frontiers):
        # self.get_logger().info('Select Goal')

        # Select the closest frontier to the current position
        min_distance = float('inf')
        closest_point = None
        if self.current_position is not None:
            for point in frontiers:
                distance = math.hypot(
                    point.x - self.current_position.x,
                    point.y - self.current_position.y
                )
                if distance < min_distance:
                    min_distance = distance
                    closest_point = point
            return closest_point
        else:
            pass

def main():
    rclpy.init()
    frontier_explorer = FrontierAnalyzer()
    rclpy.spin(frontier_explorer)
    frontier_explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()