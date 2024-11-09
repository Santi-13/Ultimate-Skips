import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point
import numpy as np
import math

class WavefrontPlanner(Node):
    def __init__(self):
        super().__init__('wavefront_planner')
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

    def find_frontiers(self, occupancy_grid):
        # Convert the 1D data array into a 2D numpy array
        data = np.array(occupancy_grid.data).reshape(
            (occupancy_grid.info.height, occupancy_grid.info.width)
        )
        frontiers = []
        resolution = occupancy_grid.info.resolution
        origin_x = occupancy_grid.info.origin.position.x
        origin_y = occupancy_grid.info.origin.position.y

        if self.current_position is not None:
            for i in range(1,10):
                x_pos = self.current_position.x
                y_pos = self.current_position.y
                data[x_pos+i*2][y_pos+i*2] 

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

class OccupancyGrid2d():
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self.__getIndex(mx, my)]

    def getSize(self):
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

        return (wx, wy)

    def worldToMap(self, wx, wy):
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        
        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")

        return (mx, my)

    def __getIndex(self, mx, my):
        return my * self.map.info.width + mx

def main():
    rclpy.init()
    wavefront_planner = WavefrontPlanner()
    rclpy.spin(wavefront_planner)
    wavefront_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()