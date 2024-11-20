import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from pynput import keyboard
import math
from slam_toolbox.srv import SaveMap
from rclpy.callback_groups import ReentrantCallbackGroup
import os
from std_msgs.msg import String  # Importar el mensaje String
from datetime import datetime

class ImprovedSimplePledgeAlgorithm(Node):
    def __init__(self):
        super().__init__('left_wall_follower')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos_profile_sensor_data
        )
        
        self.save_map_cli = self.create_client(
            SaveMap,
            'slam_toolbox/save_map',
            callback_group=self.callback_group
        )
        
        self.twist = Twist()
        self.target_distance = 0.25
        self.front_target_distance = 0.2 
        self.turn_velocity = 0.40
        self.forward_velocity = 0.025
        self.angle_tolerance = (45.0 * math.pi) / 180.0
        
        self.initial_orientation = None
        self.current_orientation = None
        self.is_following_wall = False
        self.wall_hit = False
        self.turn_counter = 0
        self.stop = False
        self.save_map_requested = False  # Nueva bandera para solicitar guardado de mapa
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10
        )

        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

        self.timer = self.create_timer(0.1, self.timer_callback)  # Ajustar el tiempo según sea necesario

    def save_map(self):
        if not self.save_map_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Servicio de guardado de mapa no disponible.')
            return
        
        # Obtener la ruta al directorio home
        home_dir = os.path.expanduser('~')
        # Crear el directorio maps en el home si no existe
        maps_dir = os.path.join(home_dir, 'maps')
        os.makedirs(maps_dir, exist_ok=True)
        
        request = SaveMap.Request()

        # Usar la ruta completa para el mapa
        map_name = os.path.join(maps_dir, 'map')

        self.get_logger().info(f'Intentando guardar el mapa con nombre: {map_name}')

        # Crear el mensaje String y asignar el nombre del mapa
        string_msg = String()
        string_msg.data = map_name

        request.name = string_msg

        future = self.save_map_cli.call_async(request)
        future.add_done_callback(self.map_save_response_callback)

    def map_save_response_callback(self, future):
        try:
            response = future.result()
            if response.result:
                self.get_logger().info('Mapa guardado exitosamente.')
            else:
                self.get_logger().error('Error al guardar el mapa. El servicio respondió con result=False.')
        except Exception as e:
            self.get_logger().error(f'Excepción al guardar el mapa: {str(e)}')

    def on_key_press(self, key):
        try:
            if key.char == 'q':
                self.stop = True
                self.save_map_requested = True  # Solicitar guardado de mapa

                # Detener el robot
                stop_twist = Twist()
                stop_twist.linear.x = 0.0
                stop_twist.angular.z = 0.0
                self.cmd_pub.publish(stop_twist)

                self.get_logger().info("Robot detenido y guardando mapa (q presionada).")

        except AttributeError:
            pass

    def timer_callback(self):
        if self.stop:
            if self.save_map_requested:
                self.save_map()
                self.save_map_requested = False  # Restablecer la solicitud
            return

        # Asegurarse de que las orientaciones no sean None
        if self.initial_orientation is not None and self.current_orientation is not None:
            difference = self.current_orientation - self.initial_orientation
            # Normalizar la diferencia de ángulo entre -pi y pi
            difference = (difference + math.pi) % (2 * math.pi) - math.pi

            # Actualizar la orientación inicial para la siguiente iteración
            self.initial_orientation = self.current_orientation

            if difference < -self.angle_tolerance:
                self.turn_counter += 1
            elif difference > self.angle_tolerance:
                self.turn_counter -= 1
        else:
            self.get_logger().warn("Orientaciones no inicializadas aún.")

    def odometry_callback(self, msg):
        # Convertir el cuaternión a ángulo yaw
        orientation_q = msg.pose.pose.orientation
        x = orientation_q.x
        y = orientation_q.y
        z = orientation_q.z
        w = orientation_q.w
        roll, pitch, yaw = self.euler_from_quaternion(x, y, z, w)

        if self.initial_orientation is None:
            self.initial_orientation = yaw
            self.get_logger().info(f"Orientación inicial establecida: {self.initial_orientation}")
        else:
            self.current_orientation = yaw

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convertir cuaternión a ángulos de Euler (roll, pitch, yaw)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = max(min(t2, +1.0), -1.0)
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw  # en radianes

    def laser_callback(self, msg):
        if self.stop:
            return

        actual_msgs = len(msg.ranges)
        
        front_ranges = [r for r in msg.ranges[0:30] + msg.ranges[int((330/360*actual_msgs)):int((359/360*actual_msgs))] if not math.isnan(r) and r > 0]
        front = min(front_ranges) if front_ranges else 10.0

        left_ranges = [r for r in msg.ranges[int((60/360*actual_msgs)):int((120/360*actual_msgs))] if not math.isnan(r) and r > 0]
        left = min(left_ranges) if left_ranges else 10.0

        if left < self.target_distance:
            if front < self.front_target_distance:
                self.turn_right()
            else:
                self.walk_straight()
        else:
            self.turn_left()

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

