# ROS2 Workspace - Autonomous Exploration and Hazmat Detection

Este repositorio contiene un workspace de ROS2 con dos paquetes principales diseñados para la exploración autónoma y la identificación de materiales peligrosos (hazmats) utilizando el TurtleBot3 Burger.

## Estructura del Proyecto

La carpeta principal `src/` contiene todos los paquetes del proyecto:

- **explorer_skips**: Implementa funcionalidades de exploración autónoma para el robot TurtleBot3 Burger. Este paquete permite que el robot navegue y explore el entorno de manera autónoma, generando un mapa de su entorno.
  
- **vision_skips**: Utiliza técnicas de inteligencia artificial para identificar materiales peligrosos (hazmats) mediante la cámara del robot. Una vez identificados, el robot puede dirigirse hacia estos puntos en el mapa.

## Requisitos Previos

Para poder ejecutar este proyecto, necesitarás tener instalado:

- ROS2 (Humble)
- TurtleBot3 slam_toolbox
- Python > 3.10
- Dependencias de inteligencia artificial para visión por computadora (OpenCV)
- Navigation 2

### Instalación de ROS2

Sigue las instrucciones oficiales de [instalación de ROS2](https://docs.ros.org/en/humble/Installation.html) para tu sistema operativo.

### Instalación de Dependencias Adicionales

Dentro del workspace, puedes instalar las dependencias adicionales usando `rosdep`:

```bash
cd /ruta/a/tu/workspace
rosdep install --from-paths src --ignore-src -r -y 
```
### Configuración del Proyecto
1.- Clona el repo en tu sistema local:
```bash
git clone git@github.com:Santi-13/Ultimate-Skips.git

cd Ultimate-Skips
```

2.-Compila el workspace de ROS2:
```bash
colcon build
```

3.- Sourcing del workspace (solo es necesario hacerlo una vez cada que se abre una nueva terminal)
```bash
source install/setup.bash
```

## Uso de los paquetes
### Explorer
Este paquete permite al TurtleBot3 explorar el entorno de manera autónoma.

Ejemplo de comando para ejecutar el nodo de exploración autónoma:
```bash
ros2 run explorer_skips nombre_del_nodo
```

### Vision
Este paquete se utiliza para la identificación de hazmats utilizando la cámara del robot. Emplea algoritmos de IA para reconocer los tipos de hazmat presentes en el mapa y permite que el robot se dirija hacia ellos.

Esto decidimos agregarlo como otro paquete, solo es necesario ejecutar el client.py desde Ubuntu. El servidor se ejecuta en una segunda computadora externa. 
```bash
python3 camara_compressed_client.py
```

