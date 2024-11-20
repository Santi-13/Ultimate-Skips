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

        # No of unknown cells for a cluster to be considered a frontier
        self.min_unknown_cells = 5

        self.goal_active = False
        self.current_position = None
        self.goal_point = None

    def map_callback(self, msg):
        self.get_logger().info(f"Received OccupancyGrid with shape: {msg.info.width}x{msg.info.height}")
        
        # Process the OccupancyGrid to find frontiers
        frontiers = self.find_frontiers(msg)    

        if frontiers:
            closest = None
            d_c = 0.0

            # Separate robot position components
            x_r = self.current_position.x
            y_r = self.current_position.y

            for frontier in frontiers:
                # Frontier components
                x_f = frontier.point.x
                y_f = frontier.point.y

                # Distance to frontier point
                d_f = np.sqrt( ( x_r - x_f )**2 + ( y_r - y_f )**2 )

                if closest:
                    # Distance to current closest frontier
                    d_c = np.sqrt( ( x_r - closest.point.x )**2 + ( y_r - closest.point.y )**2 )

                    if d_f < d_c:
                        closest = frontier

                    self.goal_pub.publish(frontier)

                    self.get_logger().info(f'Navigating to closest frontier ({closest.point.x}, {closest.point.y})')
                    self.get_logger().info(f'=======================')
                else:
                    self.goal_pub.publish(frontier)
                    
                    closest = frontier

 
    def pose_callback(self, msg):
        # self.get_logger().info('Pose Callback')
        self.current_position = msg.pose.pose.position

    def find_frontiers(self, occupancy_grid):
        # Uses wavefront planning to find the frontiers by searching around the robot
        # Convert the 1D data array into a 2D numpy array
        data = np.array(occupancy_grid.data).reshape((occupancy_grid.info.height, occupancy_grid.info.width))

        # Number of radius around the robot
        n = 5

        if self.current_position is not None:
            # Current Robot Position
            x, y = float(self.current_position.x), float(self.current_position.y)      

            # Convert robot position to grid coordinates
            resolution = occupancy_grid.info.resolution
            
            grid_x = int((x - occupancy_grid.info.origin.position.x) / resolution)
            grid_y = int((y - occupancy_grid.info.origin.position.y) / resolution)

            def check_cell(x, y, width, height):
                if 0 <= x < width and 0 <= y < height:
                    if data[y][x] == -1:
                        # a = (x + 0.5) * resolution + occupancy_grid.info.origin.position.x
                        # b = (y + 0.5) * resolution + occupancy_grid.info.origin.position.y

                        # self.get_logger().info(f'Unknown cell detected on:')
                        # self.get_logger().info(f'grid ({x}, {y})')
                        # self.get_logger().info(f'World ({a}, {b})')
                        # self.get_logger().info(f'In map, cell value is ({data[y][x]})')
                        # self.get_logger().info(f'=======================')

                        # frontier = PointStamped(
                        #         header=Header(stamp=self.get_clock().now().to_msg(), frame_id='map'),                            
                        #         point=Point(x=float(a), y=float(b), z=0.0)
                        #     )
                        # self.landmark_pub.publish(frontier)

                        if self.count_unknown_neighbors(data, x, y) >= self.min_unknown_cells:
                            return True
                    return False
                # else:
                    # self.get_logger().info("Robot position is outside the map bounds")

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

                    if check_cell(grid_x-grid_x_radius, yi,occupancy_grid.info.width,occupancy_grid.info.height):
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
                    
                    if check_cell(grid_x+grid_x_radius, yi,occupancy_grid.info.width,occupancy_grid.info.height):
                        frontier = PointStamped(
                                    header=Header(stamp=self.get_clock().now().to_msg(), frame_id='map'),                            
                                    point=Point(x=float(x+x_radius), y=float(temp_robot_y), z=0.0)
                                )

                        if frontier: 
                            frontiers.append(frontier)
                            self.landmark_pub.publish(frontier)

                for xi in range(-segment_half+grid_x, segment_half+grid_x+1):

                    
                    temp_robot_x = (xi + 0.0) * resolution + occupancy_grid.info.origin.position.x
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
                    
                    if check_cell(xi, grid_y-grid_y_radius,occupancy_grid.info.width,occupancy_grid.info.height):
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
                    
                    if check_cell(xi, grid_y+grid_y_radius,occupancy_grid.info.width,occupancy_grid.info.height):
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
                
    def count_unknown_neighbors(self, data, x, y):
        counter = 0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx != 0 or dy != 0:
                    nx, ny = x + dx, y + dy

                    if 0 <= nx < data.shape[1] and 0 <= ny < data.shape[0]:
                        # self.get_logger().info(f'AAAAAAAA: {nx}')
                        # self.get_logger().info(f'BBBBBBBB: {data.shape[0]}')
                        # self.get_logger().info(f'Neighboring:')
                        # self.get_logger().info(f'grid ({nx}, {ny})')
                        # self.get_logger().info(f'In map, cell value is ({data[ny][nx]})')
                        # self.get_logger().info(f'=======================')
                        if data[ny][nx] == -1:
                            counter += 1
                        elif data[ny][nx] == 100:
                            counter = 0
                            # self.get_logger().info(f'Vecinos tienen celda ocupada, frontera no valida')
                            return counter
        # self.get_logger().info(f'Numero de vecinos desconocidos son {counter}')
        return counter

def main():
    rclpy.init()
    wavefront_planner = WavefrontPlanner()
    rclpy.spin(wavefront_planner)
    wavefront_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()