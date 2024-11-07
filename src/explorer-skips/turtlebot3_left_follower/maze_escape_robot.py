import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
from rclpy.qos import qos_profile_sensor_data

import math

PI = 3.1415926535897

class MazeEscape(Node):
    def __init__(self):
        super().__init__('robot_cleaner')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_subscriber = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, qos_profile_sensor_data)
        
        self.vel_msg = Twist()
        self.laser_data = None  # To store the latest laser scan data

    def laser_callback(self, msg):
        # Store the latest LaserScan data for use in escape_maze
        self.get_logger().info('Laser data received')
        self.laser_data = msg

    def stop(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(self.vel_msg)
        time.sleep(1)

    def moving_forward(self, t0, current_distance, distance, speed, forward_speed, front):
        self.vel_msg.linear.x = float(forward_speed)
        self.vel_msg.angular.z = 0.0
        self.get_logger().info('Moving forward')

        while current_distance < distance:
            self.velocity_publisher.publish(self.vel_msg)
            t1 = self.get_clock().now().seconds_nanoseconds()[0]
            current_distance = speed * (t1 - t0)
        
        if front < 1.0:
            self.stop()
        time.sleep(1)

    def moving_backward(self, t0, current_distance, distance, speed, backward_speed, front):
        self.vel_msg.linear.x = float(backward_speed)
        self.vel_msg.angular.z = 0.0
        self.get_logger().info('Moving backward')

        while current_distance < (distance / 2):
            self.velocity_publisher.publish(self.vel_msg)
            t1 = self.get_clock().now().seconds_nanoseconds()[0]
            current_distance = speed * (t1 - t0)
        
        if front < 1.0:
            self.stop()
        time.sleep(1)

    def turn_cw(self, t0, current_angle, turning_speed, angle):
        angular_speed = float(round(turning_speed * 2 * PI / 360, 1))
        relative_angle = float(round(angle * 2 * PI / 360, 1))
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = -abs(angular_speed)
        self.get_logger().info('Turning clockwise')

        while current_angle < relative_angle:
            self.velocity_publisher.publish(self.vel_msg)
            t1 = self.get_clock().now().seconds_nanoseconds()[0]
            current_angle = angular_speed * (t1 - t0)

        time.sleep(1)

    def turn_ccw(self, t0, current_angle, turning_speed, angle):
        angular_speed = float(round(turning_speed * 2 * PI / 360, 1))
        relative_angle = float(round(angle * 2 * PI / 360, 1))
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = abs(angular_speed)
        self.get_logger().info('Turning counterclockwise')

        while current_angle < relative_angle:
            self.velocity_publisher.publish(self.vel_msg)
            t1 = self.get_clock().now().seconds_nanoseconds()[0]
            current_angle = angular_speed * (t1 - t0)

        time.sleep(1)

    def escape_maze(self):
        speed = 0.2
        distance = [0.10, 0.20, 0.29]
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0

        while rclpy.ok():
            if self.laser_data is None:
                self.get_logger().info('Waiting for laser scan data...')
                rclpy.spin_once(self)
                continue

            t0 = self.get_clock().now().seconds_nanoseconds()[0]
            current_distance = 0

            front = self.laser_data.ranges[1]
            left = self.laser_data.ranges[90]
            top_left = self.laser_data.ranges[45]
            right = self.laser_data.ranges[270]
            top_right = self.laser_data.ranges[315]

            no_right_wall = (right >= 2.0 and top_right >= 2.0)
            self.get_logger().info('Right wall detected' if not no_right_wall else 'Right wall not detected')

            if no_right_wall:
                if front < 1.0:
                    self.get_logger().info('Move backward')
                    self.moving_backward(t0, current_distance, distance[1], speed, -0.36, front)
                    self.get_logger().info('Turn clockwise because no right wall')
                    self.turn_cw(t0, 0, 3, 90)
                
                self.get_logger().info('Move forward because no right wall')
                if front < 0.5:
                    self.moving_forward(t0, current_distance, distance[0], speed, 0.36, front)
                else:
                    self.moving_forward(t0, current_distance, distance[2], speed, 0.36, front)
            else:
                no_front_wall = (front > 1.0 and top_left > 1.0)
                self.get_logger().info('Front wall detected' if not no_front_wall else 'Front wall not detected')

                if no_front_wall and front > 1.0:
                    if front < 0.5:
                        self.get_logger().info('Move forward because no front wall')
                        self.moving_forward(t0, current_distance, distance[0], speed, 0.36, front)
                    else:
                        self.moving_forward(t0, current_distance, distance[2], speed, 0.36, front)
                else:
                    if (left > 0.5) and ((right > 3.0) or (top_right > 3.0)):
                        if front < 1.0:
                            self.get_logger().info('Move backward')
                            self.moving_backward(t0, current_distance, distance[1], speed, -0.36, front)
                            self.get_logger().info('Turn clockwise')
                            self.turn_cw(t0, 0, 3, 90)
                    else:
                        if front < 1.0:
                            self.get_logger().info('Move backward')
                            self.moving_backward(t0, current_distance, distance[1], speed, -0.36, front)
                            self.get_logger().info('Turn counter-clockwise')
                            self.turn_ccw(t0, 0, 3, 90)

def main(args=None):
    rclpy.init(args=args)
    robot_cleaner = MazeEscape()
    robot_cleaner.escape_maze()  # Start the escape sequence
    rclpy.spin(robot_cleaner)

    robot_cleaner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
