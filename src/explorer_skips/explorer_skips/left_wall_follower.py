import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from pynput import keyboard
import math

class ImprovedSimplePledgeAlgorithm(Node):
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
        self.angle_tolerance = (45.0 * math.pi) / 180.0  # Angle at which a turn is considered
        
        self.initial_orientation = None
        self.current_orientation = None
        self.is_following_wall = False  # Flag to indicate wall following state
        self.wall_hit = False  # Flag to check if wall has been hit
        self.turn_counter = 0
        self.stop = False  # New flag to stop the robot
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10
        )

        # Start the keyboard listener for the 'q' key
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

        self.timer = self.create_timer(5, self.timer_callback)

    def timer_callback(self):
        if self.stop:
            return
        difference = self.initial_orientation - self.current_orientation
        self.initial_orientation = self.current_orientation
        if difference < -self.angle_tolerance:
            self.turn_counter += 1
        elif difference > self.angle_tolerance:
            self.turn_counter -= 1

    def on_key_press(self, key):
        try:
            if key.char == 'q':
                # Set stop flag and publish stop command
                self.stop = True
                stop_twist = Twist()
                stop_twist.linear.x = 0.0
                stop_twist.angular.z = 0.0
                
                # Log and publish the stop message
                self.get_logger().info("Stopping the robot (q pressed).")
                self.cmd_pub.publish(stop_twist)
                
        except AttributeError:
            # Ignore special keys
            pass

    def odometry_callback(self, msg):
        if self.initial_orientation is None:
            self.initial_orientation = msg.pose.pose.orientation.w  # Store initial orientation
        else:
            self.current_orientation = msg.pose.pose.orientation.w  # Store current orientation

    def laser_callback(self, msg):
        if self.stop:
            return  # Skip movement if stop flag is set

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
        if not self.stop:
            self.twist.linear.x = self.forward_velocity
            self.twist.angular.z = 0.0
            self.cmd_pub.publish(self.twist)

    def turn_right(self):
        if not self.stop:
            self.twist.linear.x = 0.0
            self.twist.angular.z = -self.turn_velocity
            self.cmd_pub.publish(self.twist)
        
    def turn_left(self):
        if not self.stop:
            self.twist.linear.x = 0.0
            self.twist.angular.z = +self.turn_velocity
            self.cmd_pub.publish(self.twist)

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