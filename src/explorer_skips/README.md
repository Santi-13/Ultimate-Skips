# explorer_skips

Este paquete contiene los scripts y configuraciones necesarios para ejecutar la exploración autónoma en el TurtleBot3 Burger, utilizando diversas técnicas de navegación y planeación.

## Estructura del Paquete

- **frontier_analyzer.py**: Script para analizar las fronteras no exploradas en el entorno. Identifica áreas aún no exploradas y determina posibles ubicaciones de navegación.

- **landmark_marker.py**: Coloca marcadores en el entorno para ayudar en la navegación y ubicación de referencias espaciales (landmarks).

- **left_wall_follower.py**: Implementa un algoritmo de seguimiento de paredes para que el robot navegue siguiendo una pared a su izquierda.

- **nav_goal_sender.py**: Envía objetivos de navegación al sistema de control del TurtleBot3 para dirigirse a áreas específicas.

- **wavefront_planner.py**: Genera un plan de navegación basado en el algoritmo de propagación de onda (Wavefront) para determinar la ruta óptima hacia el objetivo.

## Archivos de Configuración

- **config/navigation.yaml**: Parámetros de configuración para el sistema de navegación, ajustables según las necesidades del entorno.
- **config/slam.yaml**: Configuración del SLAM (Simultaneous Localization and Mapping) para la creación y uso del mapa del entorno.

## Archivos de Lanzamiento

- **launch/exploration.launch.py**: Archivo de lanzamiento para iniciar la exploración autónoma utilizando el paquete `explorer_skips`.
- **launch/left_wall_exploration.launch.py**: Archivo de lanzamiento para iniciar la exploración utilizando el algoritmo de seguimiento de pared izquierda.

## Instalación

Este paquete es parte de un workspace ROS2. Asegúrate de haber configurado y compilado el workspace principal antes de ejecutar los siguientes comandos.

### Paso 1: Compilar el Workspace

```bash
cd /ruta/a/tu/workspace
colcon build --packages-select explorer_skips
```

### Paso 2: Sourcing del Workspace
Solo es necesario hacer esto una vez en cada terminal nueva.
```bash
source install/setup.bash
```

## Uso
Para iniciar la exploración autónoma, primero es necesario correr el slam_toolbox con el siguiente comando:
```bash
ros2 launch slam_toolbox online_async_launch.py
```

Despues podemos probar algun algortimo de navegación como el seguidor de pared izquierda:

```bash
ros2 launch explorer_skips left_wall_exploration.launch.py
```
