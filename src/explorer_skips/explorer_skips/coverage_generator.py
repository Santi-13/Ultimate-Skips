import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PointStamped
import math
import random
import numpy as np

class CoverageGenerator(Node):
    def __init__(self):
        super().__init__('coverage_generator')
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        self.coverage_pub = self.create_publisher(
            PointStamped,
            'coverage_points',
            10)
        self.map_data = None
        self.resolution = None
        self.origin_x = None
        self.origin_y = None
        self.width = None
        self.height = None

    def map_callback(self, msg):
        if self.map_data is None:
            self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
            self.resolution = msg.info.resolution
            self.origin_x = msg.info.origin.position.x
            self.origin_y = msg.info.origin.position.y
            self.width = msg.info.width
            self.height = msg.info.height
            self.generate_coverage_points()

    def generate_coverage_points(self):
        target_points = 15  # Adjust as needed
        coverage_radius = 0.4  # Adjust as needed
        
        coverage_points = []
        
        while len(coverage_points) < target_points:
            x = random.uniform(-self.width * self.resolution / 2, self.width * self.resolution / 2)
            y = random.uniform(-self.height * self.resolution / 2, self.height * self.resolution / 2)
            
            if self.is_valid_point(x, y) and not self.is_covered(x, y, coverage_radius):
                coverage_points.append((x, y))
                self.publish_point(x, y)
                print(f"New point added: ({x}, {y})")
        
        self.get_logger().info(f"Generated {len(coverage_points)} coverage points")

    def is_valid_point(self, x, y):
        map_x = int((x - self.origin_x) / self.resolution)
        map_y = int((y - self.origin_y) / self.resolution)
        
        if map_x < 0 or map_x >= self.width or map_y < 0 or map_y >= self.height:
            return False
        
        return self.map_data[map_y, map_x] != -1  # -1 represents unknown space

    def is_covered(self, x, y, radius):
        start_angle = 0
        end_angle = 2 * math.pi
        angle_step = math.pi / 180  # 1 degree steps
        
        for angle in np.arange(start_angle, end_angle, angle_step):
            dx = radius * math.cos(angle)
            dy = radius * math.sin(angle)
            
            end_x = x + dx
            end_y = y + dy
            
            if not self.is_valid_point(end_x, end_y):
                continue
            
            map_end_x = int((end_x - self.origin_x) // self.resolution)
            map_end_y = int((end_y - self.origin_y) // self.resolution)
            
            if self.map_data[map_end_y, map_end_x] > 50:  # 50 is the threshold for occupied cells
                return True
        
        return False

    def publish_point(self, x, y):
        point_msg = PointStamped()
        point_msg.header.frame_id = "map"
        point_msg.point.x = x
        point_msg.point.y = y
        self.coverage_pub.publish(point_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CoverageGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
