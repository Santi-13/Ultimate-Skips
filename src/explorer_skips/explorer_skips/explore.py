import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from std_msgs.msg import Header
import numpy as np
import math

class WavefrontPlanner(Node):
    def __init__(self):
        super().__init__('wavefront_planner')
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            10
        )
        
        # self.goal_pub = self.create_publisher(
        #     PointStamped,
        #     '/unknown_frontier_goal',
        #     10
        # )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.landmark_pub = self.create_publisher(
            PointStamped,
            '/landmarks',
            10
        )

        # No of unknown cells for a cluster to be considered a frontier
        self.min_unknown_cells = 5

        self.current_position = None
        self.last_sent_goal = None
        self.minimum_frontier_distance = 0.2  # Adjust this value as needed

    def pose_callback(self, msg):
        # self.get_logger().info('Pose Callback')
        self.current_position = msg.pose.pose.position

    def map_callback(self, msg):
        self.get_logger().info(f"Received OccupancyGrid with shape: {msg.info.width}x{msg.info.height}")
        
        frontiers = self.find_frontiers(msg)

        frontiers = [
            frontier for frontier in frontiers
            if self.distance_to_robot(frontier.point) > self.minimum_frontier_distance
        ]
        
        if frontiers:
            closest = min(frontiers, key=lambda frontier: self.distance_to_robot(frontier.point))
            
            if not self.last_sent_goal or self.distance_between_goals(closest, self.last_sent_goal) > 0.1:
                self.send_goal(closest)
                self.last_sent_goal = closest

        else:
            self.get_logger().info("Exploration completed - no more frontiers found")

    def send_goal(self, point):
    
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = point.point.x
        msg.pose.position.y = point.point.y
        msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Sending goal to ({msg.pose.position.x}, {msg.pose.position.y})')

    def distance_to_robot(self, point):
        return math.sqrt((point.x - self.current_position.x)**2 + (point.y - self.current_position.y)**2)

    def distance_between_goals(self, goal1, goal2):
        return math.sqrt((goal1.point.x - goal2.point.x)**2 + (goal1.point.y - goal2.point.y)**2)    

    def check_cell(self, x, y, width, height, data):
        if 0 <= x < width and 0 <= y < height:
            if data[y][x] == -1 and self.is_frontier_cell(data, x, y):
                return True
        return False
    
    def find_frontiers(self, occupancy_grid):
        # Uses wavefront planning to find the frontiers by searching around the robot
        # Convert the 1D data array into a 2D numpy array
        data = np.array(occupancy_grid.data).reshape((occupancy_grid.info.height, occupancy_grid.info.width))

        # Number of radius around the robot
        n = 8

        if self.current_position is not None:
            # Current Robot Position
            x, y = float(self.current_position.x), float(self.current_position.y)      

            # Convert robot position to grid coordinates
            resolution = occupancy_grid.info.resolution
            
            grid_x = int((x - occupancy_grid.info.origin.position.x) / resolution)
            grid_y = int((y - occupancy_grid.info.origin.position.y) / resolution)

            # Frontiers array
            frontiers = []
            for i in range(1,n):
                grid_x_radius = 2*i
                grid_y_radius = 2*i

                x_radius = (grid_x_radius + 0.0) * resolution 
                y_radius = (grid_y_radius + 0.0) * resolution 
                
                # self.get_logger().info(f'=======================')
                # self.get_logger().info(f'map dimensions: ({occupancy_grid.info.width}, {occupancy_grid.info.height})')
                # self.get_logger().info(f'=======================')
                # self.get_logger().info(f'Circumference no {i}')
                # self.get_logger().info(f'Currently on: ')
                # self.get_logger().info(f'World ({x}, {y})')
                # self.get_logger().info(f'Grid ({grid_x}, {grid_y})')
                # self.get_logger().info(f'=======================')
                segment_half = int(4*i/2)

                a = (grid_y + 0.5) * resolution + occupancy_grid.info.origin.position.y
                b = (grid_x + 0.5) * resolution + occupancy_grid.info.origin.position.x

                frontier = PointStamped(
                                header=Header(stamp=self.get_clock().now().to_msg(), frame_id='map'),                            
                                point=Point(x=float(b), y=float(a), z=0.0)
                            )
                self.landmark_pub.publish(frontier)

                for yi in range(-segment_half + grid_y, segment_half + grid_y+1):
                    temp_robot_y = (yi + 0.50) * resolution + occupancy_grid.info.origin.position.y
                    # self.get_logger().info(f'checking grid ({grid_x-grid_x_radius}, {yi})')
                    # self.get_logger().info(f'Marker on ({x-x_radius}, {temp_robot_y})')
                    # self.get_logger().info(f'In map, cell value is ({data[yi][grid_x-grid_x_radius]})')
                    # self.get_logger().info(f'=======================')
                    # self.get_logger().info(f'checking grid ({grid_x+grid_x_radius}, {yi})')
                    # self.get_logger().info(f'Marker on ({x+x_radius}, {temp_robot_y})')
                    # self.get_logger().info(f'In map, cell value is ({data[yi][grid_x+grid_x_radius]})')
                    # self.get_logger().info(f'=======================')

                    # self.landmark_pub.publish(PointStamped(
                    #         header=Header(stamp=self.get_clock().now().to_msg(), frame_id='map'),                            
                    #         point=Point(x=float(x-x_radius), y=float(temp_robot_y), z=0.0)
                    #     ))

                    if self.check_cell(grid_x-grid_x_radius, yi,occupancy_grid.info.width,occupancy_grid.info.height, data):
                        frontier = PointStamped(
                                    header=Header(stamp=self.get_clock().now().to_msg(), frame_id='map'),                            
                                    point=Point(x=float(x-x_radius), y=float(temp_robot_y), z=0.0)
                                )
                    
                        if frontier:                         
                            frontiers.append(frontier)
                            self.landmark_pub.publish(frontier)
                    
                    # self.landmark_pub.publish(PointStamped(
                    #         header=Header(stamp=self.get_clock().now().to_msg(), frame_id='map'),                            
                    #         point=Point(x=float(x+x_radius), y=float(temp_robot_y), z=0.0)
                    #     ))
                    
                    if self.check_cell(grid_x+grid_x_radius, yi,occupancy_grid.info.width,occupancy_grid.info.height, data):
                        frontier = PointStamped(
                                    header=Header(stamp=self.get_clock().now().to_msg(), frame_id='map'),                            
                                    point=Point(x=float(x+x_radius), y=float(temp_robot_y), z=0.0)
                                )

                        if frontier: 
                            frontiers.append(frontier)
                            self.landmark_pub.publish(frontier)

                for xi in range(-segment_half+grid_x, segment_half+grid_x+1):

                    
                    temp_robot_x = (xi + 0.50) * resolution + occupancy_grid.info.origin.position.x
                    # self.get_logger().info(f'checking grid ({xi}, {grid_y-grid_y_radius})')
                    # self.get_logger().info(f'Marker on ({temp_robot_x}, {y-y_radius})')
                    # self.get_logger().info(f'In map, cell value is ({data[grid_y-grid_y_radius][xi]})')
                    # self.get_logger().info(f'=======================')
                    # self.get_logger().info(f'checking grid ({xi}, {grid_y+grid_y_radius})')
                    # self.get_logger().info(f'Marker on ({temp_robot_x}, {y+y_radius})')
                    # self.get_logger().info(f'In map, cell value is ({data[grid_y+grid_y_radius][xi]})')
                    # self.get_logger().info(f'=======================')

                    # self.landmark_pub.publish(PointStamped(
                    #         header=Header(stamp=self.get_clock().now().to_msg(), frame_id='map'),                            
                    #         point=Point(x=float(temp_robot_x), y=float(y-y_radius), z=0.0)
                    #     ))
                    
                    if self.check_cell(xi, grid_y-grid_y_radius,occupancy_grid.info.width,occupancy_grid.info.height, data):
                        frontier = PointStamped(
                                    header=Header(stamp=self.get_clock().now().to_msg(), frame_id='map'),                            
                                    point=Point(x=float(temp_robot_x), y=float(y-y_radius), z=0.0)
                                )

                        if frontier: 
                            frontiers.append(frontier)
                            self.landmark_pub.publish(frontier)
                    
                    # self.landmark_pub.publish(PointStamped(
                    #         header=Header(stamp=self.get_clock().now().to_msg(), frame_id='map'),                            
                    #         point=Point(x=float(temp_robot_x), y=float(y+y_radius), z=0.0)
                    #     ))
                    
                    if self.check_cell(xi, grid_y+grid_y_radius,occupancy_grid.info.width,occupancy_grid.info.height, data):
                        frontier = PointStamped(
                                    header=Header(stamp=self.get_clock().now().to_msg(), frame_id='map'),                            
                                    point=Point(x=float(temp_robot_x), y=float(y+y_radius), z=0.0)
                                )
                                
                        if frontier: 
                            frontiers.append(frontier)
                            self.landmark_pub.publish(frontier)
            
            return frontiers
        else:
            self.get_logger().info("No robot position available")
                
    def is_frontier_cell(self, data, x, y):
        # A frontier cell is unknown (-1) and adjacent to at least one free cell (0)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = x + dx, y + dy
                if dx == 0 and dy == 0:
                    continue
                if 0 <= nx < data.shape[1] and 0 <= ny < data.shape[0]:
                    if data[ny][nx] == 0:
                        return True
        return False

def main():
    rclpy.init()
    wavefront_planner = WavefrontPlanner()
    rclpy.spin(wavefront_planner)
    wavefront_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()