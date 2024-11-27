import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class LaserScanFilterNode(Node):
    def __init__(self):
        super().__init__('laser_scan_filter_node')
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=self.sensor_qos)
        self.publisher_ = self.create_publisher(
            LaserScan,
            '/scan_filtered',
            qos_profile=self.sensor_qos)

        # Define the angular bounds in radians
        self.min_angle = -30.0 * (math.pi / 180.0)  # -30 degrees
        self.max_angle = 30.0 * (math.pi / 180.0)   # 30 degrees

    def scan_callback(self, msg):
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = list(msg.ranges)
        filtered_scan.intensities = list(msg.intensities)

        for i in range(len(filtered_scan.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            if angle < self.min_angle or angle > self.max_angle:
                filtered_scan.ranges[i] = float('nan')
                if filtered_scan.intensities:
                    filtered_scan.intensities[i] = 0.0

        self.publisher_.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
