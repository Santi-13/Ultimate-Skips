#include <micro_ros_arduino.h>
#include <ESP32Servo.h> 
#include <sensor_msgs/msg/imu.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

Servo servo1;
Servo servo2;

rcl_subscription_t imu_subscriber;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define SERVO_PIN1 18
#define SERVO_PIN2 19

const float initialServo1Position = 0.0;
const float initialServo2Position = 10.0;

float toRadians(float degrees) {
  return degrees * (M_PI / 180.0);
}

void error_loop() {
  Serial.println("Error occurred, entering error loop");
  while(1) {
    delay(100);
  }
}

void imu_callback(const void *msgin) {
  const sensor_msgs__msg__Imu *imu_msg = (const sensor_msgs__msg__Imu *)msgin;
  
  // Mensaje de depuración para indicar que el callback fue llamado
  Serial.println("IMU callback triggered");

  // Tomar valores de orientación del mensaje de IMU
  float x_val = imu_msg->orientation.x;
  float y_val = imu_msg->orientation.y;
  float z_val = imu_msg->orientation.z;

  // Aplicar la rotación y multiplicación
  float x_rot = -y_val * 2;
  float y_rot = x_val * 2;
  float z_rot = z_val * 2;

  float x_rad = toRadians(x_rot);
  float y_rad = toRadians(y_rot);
  float z_rad = toRadians(z_rot);

  float theta1d = atan2(y_rad, z_rad);
  float theta2d = atan2((-x_rad * y_rad), (pow(y_rad, 2) + pow(z_rad, 2)));

  float theta1_deg = theta1d * (180.0 / M_PI);
  float theta2_deg = theta2d * (180.0 / M_PI);

  float servo1_position = initialServo1Position + theta1_deg;
  float servo2_position = initialServo2Position + theta2_deg;

  servo1_position = constrain(servo1_position, 0, 180);
  servo2_position = constrain(servo2_position, 0, 180);

  servo1.write(servo1_position);
  servo2.write(servo2_position);

  // Mensajes de depuración para mostrar los valores calculados
  Serial.print("x_rot: ");
  Serial.print(x_rot);
  Serial.print(", y_rot: ");
  Serial.print(y_rot);
  Serial.print(", z_rot: ");
  Serial.println(z_rot);
  Serial.print("Servo1 Position: ");
  Serial.println(servo1_position);
  Serial.print("Servo2 Position: ");
  Serial.println(servo2_position);
  Serial.println("---------------------------");
}

void setup() {
  set_microros_transports();
  Serial.begin(115200);
  
  // Mensaje de depuración al inicio del setup
  Serial.println("Setting up IMU Subscriber Node...");

  servo1.attach(SERVO_PIN1);
  servo2.attach(SERVO_PIN2);

  servo1.write(initialServo1Position);
  servo2.write(initialServo2Position);

  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("Failed to initialize rclc support");
    error_loop();
  } else {
    Serial.println("rclc support initialized");
  }

  if (rclc_node_init_default(&node, "imu_subscriber_node", "", &support) != RCL_RET_OK) {
    Serial.println("Failed to initialize rclc node");
    error_loop();
  } else {
    Serial.println("Node initialized successfully");
  }

  if (rclc_subscription_init_default(
      &imu_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/imu") != RCL_RET_OK) {
    Serial.println("Failed to initialize IMU subscription");
    error_loop();
  } else {
    Serial.println("IMU subscription initialized");
  }

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    Serial.println("Failed to initialize rclc executor");
    error_loop();
  } else {
    Serial.println("Executor initialized");
  }

  if (rclc_executor_add_subscription(&executor, &imu_subscriber, &imu_msg, &imu_callback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("Failed to add subscription to executor");
    error_loop();
  } else {
    Serial.println("Subscription added to executor");
  }

  Serial.println("IMU Subscriber Node initialized and waiting for data...");
}

void loop() {
  // Remove the delay to allow continuous spinning
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // Reduced timeout for quicker response
}
