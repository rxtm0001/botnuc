from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de detección de cámara
        Node(
            package='camera_detection',  # Asegúrate de que el nombre del paquete sea correcto
            executable='camera_detection_node.py',  # O el nombre del nodo correcto si no lleva .py
            output='screen'
        ),
        # Nodo de procesamiento de puntos de la bandera
        Node(
            package='camera_detection',  # Cambia al nombre correcto de tu paquete para este nodo
            executable='flag_point_node.py',  # Nombre del archivo Python para el nodo
            output='screen'
        ),
        # Nodo de cálculo de posición de la bandera
        Node(
            package='camera_detection',  # Cambia al nombre correcto de tu paquete para este nodo
            executable='flag_position_node.py',  # Nombre del archivo Python para el nodo
            output='screen'
        ),
    ])
