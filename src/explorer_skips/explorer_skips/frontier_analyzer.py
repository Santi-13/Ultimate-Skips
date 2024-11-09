import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import numpy as np

class FrontierAnalyzer(Node):
    def __init__(self):
        super().__init__('frontier_analyzer')
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        self.goal_pub = self.create_publisher(
            Point,
            'unknown_frontier_goal',
            10
        )

    def map_callback(self, msg):
        # Process the OccupancyGrid to find frontiers
        frontiers = self.find_frontiers(msg)
        if frontiers:
            # Choose a frontier point (e.g., the closest one)
            goal_point = self.select_goal(frontiers)
            # Publish the goal point
            self.goal_pub.publish(goal_point)
            self.get_logger().info(f'Published goal point: x={goal_point.x}, y={goal_point.y}')
        else:
            self.get_logger().info('No frontiers found.')

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
        # Implement selection logic here
        # For simplicity, select the first frontier point
        return frontiers[0]

def main():
    rclpy.init()
    frontier_explorer = FrontierAnalyzer()
    rclpy.spin(frontier_explorer)
    frontier_explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()