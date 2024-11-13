import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
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
            PointStamped,
            'unknown_frontier_goal',
            10
        )

        self.landmark_pub = self.create_publisher(
            PointStamped,
            'landmarks',
            10
        )


        self.min_unknown_cells = 7

        self.goal_active = False
        self.current_position = None
        self.goal_point = None

    def map_callback(self, msg):
        # Process the OccupancyGrid to find frontiers
        frontiers = self.find_frontiers(msg)
 

    def pose_callback(self, msg):
        # self.get_logger().info('Pose Callback')
        self.current_position = msg.pose.pose.position

    def find_frontiers(self, occupancy_grid):
        # Uses wavefront planning to find the frontiers by searching around the robot
        # Convert the 1D data array into a 2D numpy array
        data = np.array(occupancy_grid.data).reshape(
            ( occupancy_grid.info.height, occupancy_grid.info.width )
        )
        
        # Number of radius around the robot
        n = 10

        # Current Robot Position
        if self.current_position is not None:
            x, y = int(self.current_position.x), int(self.current_position.y)      

            def check_cell(x, y):
                if data[x][y] == -1:
                    if self.count_unknown_neighbors(data, x, y) >= self.min_unknown_cells:
                        return PointStamped(
                            header=Header(stamp=self.get_clock().now().to_msg(), frame_id='map'),                            
                            point=Point(x=float(x), y=float(y), z=0.0)
                        )
                return None
        
            # Frontiers array
            frontiers = []
            for i in range(1,n):
                upper_bound_x = x + 2*i
                upper_bound_y = y + 2*i

                segment_half = int(4*n/2)

                for yi in range(-segment_half+y, segment_half+y+1):
                    frontier = check_cell(-upper_bound_x, yi)
                    if frontier: 
                        self.get_logger().info("Frontera Encontrada")
                        frontiers.append(frontier)
                        self.landmark_pub.publish(frontier)
                    
                    frontier = check_cell(upper_bound_x, yi)
                    if frontier: 
                        self.get_logger().info("Frontera Encontrada")
                        frontiers.append(frontier)
                        self.landmark_pub.publish(frontier)

                for xi in range(-segment_half+x, segment_half+x+1):
                    frontier = check_cell(xi, -upper_bound_y)
                    if frontier: 
                        self.get_logger().info("Frontera Encontrada")
                        frontiers.append(frontier)
                        self.landmark_pub.publish(frontier)
                    
                    frontier = check_cell(xi, upper_bound_y)
                    if frontier: 
                        self.get_logger().info("Frontera Encontrada")

                        frontiers.append(frontier)
                        self.landmark_pub.publish(frontier)
                
    def count_unknown_neighbors(self, data, x, y):
        counter = 0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx != 0 or dy != 0:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < data.shape[0] and 0 <= ny < data.shape[1]:
                        if data[nx][ny] == -1:
                            counter += 1
        return counter

def main():
    rclpy.init()
    wavefront_planner = WavefrontPlanner()
    rclpy.spin(wavefront_planner)
    wavefront_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()