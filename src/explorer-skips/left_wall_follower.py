from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import rclpy
import math

class LeftWallFollower(Node):
    def __init__(self):
        super().__init__('left_wall_follower')
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos_profile_sensor_data
        )
        
        self.twist = Twist()
        self.target_distance = 0.25  # Distance to maintain from walls
        self.front_target_distance = 0.2 
        self.turn_velocity = 0.40  # Velocity for turning
        self.forward_velocity = 0.05  # Forward velocity
        self.angle_tolerance = (45.0 * math.pi) / 180.0 # Angle at which a turn is considered
        
        self.initial_orientation = None
        self.current_orientation = None
        self.is_following_wall = False  # Flag to indicate wall following state
        self.wall_hit = False  # Flag to check if wall has been hit
        self.turn_counter = 0
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10
        )

        self.timer = self.create_timer(5, self.timer_callback)

    def timer_callback(self):
        difference = self.initial_orientation - self.current_orientation
        # self.get_logger().info(f"=================================")
        # self.get_logger().info(f"Initial angle: {self.initial_orientation*180/math.pi}")
        # self.get_logger().info(f"Current angle: {self.current_orientation*180/math.pi}")
        # self.get_logger().info(f"Angular Difference: {difference*180/math.pi}")
        # self.get_logger().info(f"Angular Counter: {self.turn_counter}")
        # self.get_logger().info(f"=================================\n")

        self.initial_orientation = self.current_orientation
        if difference < -self.angle_tolerance:
            self.turn_counter += 1
        elif difference > self.angle_tolerance:
            self.turn_counter -= 1

    def odometry_callback(self, msg):
        if self.initial_orientation is None:
            self.initial_orientation = msg.pose.pose.orientation.w  # Store initial orientation
        else:
            self.current_orientation = msg.pose.pose.orientation.w  # Store current orientation

    def laser_callback(self, msg):
        actual_msgs = len(msg.ranges)
        
        front_ranges = [r for r in msg.ranges[0:30] + msg.ranges[int((330/360*actual_msgs)):int((359/360*actual_msgs))] if not math.isnan(r) and r > 0]
        front = min(front_ranges) if front_ranges else 10.0

        left_ranges = [r for r in msg.ranges[int((60/360*actual_msgs)):int((120/360*actual_msgs))] if not math.isnan(r) and r > 0]
        left = min(left_ranges) if left_ranges else 10.0

        self.get_logger().info(f"=================================")
        self.get_logger().info(f"Front Distance: {front}")
        self.get_logger().info(f"Left Distance: {left}")
        self.get_logger().info(f"=================================\n")

        if left < self.target_distance:
            if front < self.front_target_distance:
                self.turn_right()  # Obstacle in front, turn right
            else:
                self.walk_straight()  # Nothing in front, move forward
        else:
            self.turn_left()  # No wall on the left, turn left

    def walk_straight(self):
        self.twist.linear.x = self.forward_velocity
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)

    def turn_right(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = -self.turn_velocity
        self.cmd_pub.publish(self.twist)
        
    def turn_left(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = +self.turn_velocity
        self.cmd_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    left_wall_follower = LeftWallFollower()
    try:
        rclpy.spin(left_wall_follower)
    except KeyboardInterrupt:
        pass
    finally:
        left_wall_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()