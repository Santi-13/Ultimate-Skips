import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from rclpy.duration import Duration

class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_sub = self.create_subscription(
            PointStamped,
            'unknown_frontier_goal',
            self.goal_callback,
            10
        )
        self.current_goal_handle = None

    def goal_callback(self, msg):
        x = msg.point.x
        y = msg.point.y

        if self.current_goal_handle:
            self.cancel_current_goal()
        
        self.send_goal(x, y, 0.0)

    def send_goal(self, position_x, position_y, orientation_z):
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for NavigateToPose action server...')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(position_x)
        goal_msg.pose.pose.position.y = float(position_y)
        goal_msg.pose.pose.position.z = 0.0
        
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = float(orientation_z)
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info('Sending navigation goal...')
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        self.current_goal_handle = send_goal_future.result()
        
        if not self.current_goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.current_goal_handle = None
            return
            
        self.get_logger().info('Goal accepted')
        
        result_future = self.current_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        status = result_future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().info(f'Navigation finished with status: {status}')
        
        self.current_goal_handle = None

    def cancel_current_goal(self):
        cancel_goal_future = self.current_goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_goal_future)
        self.get_logger().info('Current goal canceled')

def main():
    rclpy.init()
    
    nav_goal_sender = NavGoalSender()
    rclpy.spin(nav_goal_sender)

    nav_goal_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
