import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import math

class ImprovedSimplePledgeAlgorithm(Node):
    def __init__(self):
        super().__init__('improved_simple_pledge_algorithm')
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos_profile_sensor_data
        )
        
        self.twist = Twist()
        self.target_distance = 0.2  # Distance to maintain from walls
        self.turn_velocity = 0.40  # Velocity for turning
        self.forward_velocity = 0.05  # Forward velocity
        self.angle_tolerance = ( 45.0 * math.pi ) / 180.0 # Angle at which a turn is considered
        
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
        self.get_logger().info(f"=================================")
        self.get_logger().info(f"Initial angle: {self.initial_orientation*180/math.pi}")
        self.get_logger().info(f"Current angle: {self.current_orientation*180/math.pi}")
        self.get_logger().info(f"Angular Difference: {difference*180/math.pi}")
        self.get_logger().info(f"Angular Counter: {self.turn_counter}")
        self.get_logger().info(f"=================================\n")


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
        # Process laser scan data
        actual_msgs = len(msg.ranges)
        
        front_ranges = [r for r in msg.ranges[0:30] + msg.ranges[int((330/360*actual_msgs)):int((359/360*actual_msgs))] if not math.isnan(r) and r > 0]
        front = min(front_ranges) if front_ranges else 10.0
        

        right_ranges = [r for r in msg.ranges[int((240/360*actual_msgs)):int((300/360*actual_msgs))] if not math.isnan(r) and r > 0]
        right = min(right_ranges) if right_ranges else 10.0
        
        #self.get_logger().info(f"Front distance: {front:.2f}, Right distance: {right:.2f}")
        
        if not self.is_following_wall:
            if front < self.target_distance:
                self.hit_wall()
            else:
                self.walk_straight()
        else:
            self.follow_wall(front, right)

    def walk_straight(self):
        self.twist.linear.x = self.forward_velocity
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)
        #self.get_logger().info("Walking straight")

    def hit_wall(self):
        self.wall_hit = True
        self.turn_left()
        self.is_following_wall = True
        #self.get_logger().info("Wall hit, turning right")

    def turn_right(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = -self.turn_velocity
        self.cmd_pub.publish(self.twist)
        #self.get_logger().info("Turning right")
        
    def turn_left(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = +self.turn_velocity
        self.cmd_pub.publish(self.twist)
        #self.get_logger().info("Turning left")

    def follow_wall(self, front_distance, right_distance):
        if front_distance < self.target_distance:
            self.turn_left()  # Turn left if there's an obstacle ahead
        elif right_distance < (2 * self.target_distance):
            error = right_distance - self.target_distance
            angular_velocity = max(-self.turn_velocity, min(self.turn_velocity, -error * 2))
            
            self.twist.linear.x = self.forward_velocity
            self.twist.angular.z = 1.0*angular_velocity
            
            self.cmd_pub.publish(self.twist)
            
            current_orientation = self.current_orientation
            angle_diff = self.calculate_angle_difference(current_orientation, self.initial_orientation)
            
            if abs(angle_diff) < 0.01:  # Check if angle difference is close to zero
                self.is_following_wall = False
                #self.get_logger().info("Finished following wall, resuming straight path")
            
            #self.get_logger().info(f"Following wall (right), Angle difference: {angle_diff:.2f}")
        
        elif self.turn_counter != 0:
            self.turn_right()
        else:
            self.walk_straight()  # Continue straight if no obstacles detected

    def calculate_angle_difference(self, current, initial):
        return math.atan2(math.sin(current - initial), math.cos(current - initial))

def main(args=None):
    rclpy.init(args=args)
    improved_simple_pledge_algorithm = ImprovedSimplePledgeAlgorithm()
    try:
        rclpy.spin(improved_simple_pledge_algorithm)
    except KeyboardInterrupt:
        pass
    finally:
        improved_simple_pledge_algorithm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
