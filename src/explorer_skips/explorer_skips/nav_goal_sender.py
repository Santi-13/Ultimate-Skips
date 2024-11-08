import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from rclpy.duration import Duration

class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_sub = self.create_subscription(
            Point,
            'unknown_frontier_goal',
            self.goal_callback,
            10
        )
        
    def goal_callback(self,msg):
        x = msg.x
        y = msg.y

        self.send_goal(x,y,0.0)

    def send_goal(self, position_x, position_y, orientation_z):
        # Esperar a que el servidor de acción esté disponible
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Esperando al servidor de NavigateToPose...')

        # Crear el mensaje de pose objetivo
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Establecer la posición objetivo
        goal_msg.pose.pose.position.x = float(position_x)
        goal_msg.pose.pose.position.y = float(position_y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Establecer la orientación (usando quaternion simplificado)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = float(orientation_z)
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info('Enviando goal de navegación...')
        
        # Enviar el goal y obtener el future
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rechazado')
            return
            
        self.get_logger().info('Goal aceptado')
        
        # Obtener el resultado
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        status = result_future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal alcanzado con éxito!')
        else:
            self.get_logger().info(f'Navegación terminada con status: {status}')

def main():
    rclpy.init()
    
    nav_goal_sender = NavGoalSender()
    rclpy.spin(nav_goal_sender)

    nav_goal_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()