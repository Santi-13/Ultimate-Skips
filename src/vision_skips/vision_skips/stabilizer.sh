#!/bin/bash

# Configura el entorno de ROS 2
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/setup.bash

# Inicia el nodo de la cámara
ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[128,96]" -p fra>

# Espera un momento para asegurarse de que el nodo de la cámara inicie
sleep 5

# Configura permisos para el dispositivo USB
sudo chmod 777 /dev/ttyUSB0

# Inicia el agente de micro-ROS
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0