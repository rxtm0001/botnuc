#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import re
from collections import deque

class FlagPositionNode(Node):

    def __init__(self):
        super().__init__('flag_position_node')

        # Suscribirse a /flag_detector
        self.flag_sub = self.create_subscription(
            String,
            '/flag_detector',
            self.flag_callback,
            10
        )

        # Suscribirse a /odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publicador para /flag_position
        self.flag_pub = self.create_publisher(
            PoseStamped,
            '/flag_position',
            10
        )

        # Almacenar la última distancia y la última odometría
        self.last_distance = None
        self.last_odom = None

        # Bandera para indicar si la bandera está siendo detectada
        self.flag_detected = False

        # Almacenar la última posición calculada de la bandera
        self.last_flag_pose = None

        # Tiempo del último mensaje recibido del detector de banderas
        self.last_flag_msg_time = self.get_clock().now()

        # Temporizador para verificar si la bandera sigue siendo detectada
        self.timer = self.create_timer(0.1, self.check_flag_timeout)

        # Buffers para la media móvil de 300 posiciones
        self.flag_position_buffer_x = deque(maxlen=300)  # Para la posición X
        self.flag_position_buffer_y = deque(maxlen=300)  # Para la posición Y

    def flag_callback(self, msg):
        # Actualizar el tiempo del último mensaje recibido
        self.last_flag_msg_time = self.get_clock().now()

        # Extraer la distancia desde el mensaje
        distance = self.extract_distance(msg.data)
        if distance is not None:
            self.last_distance = distance
            self.flag_detected = True
            # Intentar calcular la posición de la bandera
            self.calculate_flag_position()
        else:
            # Si no se pudo extraer la distancia, considerar que la bandera no está siendo detectada
            self.flag_detected = False

    def odom_callback(self, msg):
        self.last_odom = msg
        if self.flag_detected:
            # Recalcular la posición de la bandera si está siendo detectada
            self.calculate_flag_position()
        else:
            # Si no se detecta la bandera, seguir publicando la última posición conocida
            if self.last_flag_pose is not None:
                self.flag_pub.publish(self.last_flag_pose)

    def extract_distance(self, data_str):
        # Usar una expresión regular para extraer el número
        match = re.search(r'(\d+\.\d+|\d+)', data_str)
        if match:
            return float(match.group(0))
        else:
            # No se pudo extraer la distancia
            return None

    def calculate_flag_position(self):
        if self.last_distance is not None and self.last_odom is not None:
            # Obtener la posición y orientación del robot
            robot_pose = self.last_odom.pose.pose
            x_r = robot_pose.position.x
            y_r = robot_pose.position.y
            # Obtener la orientación del robot en ángulo yaw
            orientation_q = robot_pose.orientation
            yaw = self.quaternion_to_yaw(orientation_q)

            # Calcular la posición de la bandera en el marco del mapa
            x_f = x_r + self.last_distance * math.cos(yaw)
            y_f = y_r + self.last_distance * math.sin(yaw)

            # Almacenar la nueva posición en los buffers para la media móvil
            self.flag_position_buffer_x.append(x_f)
            self.flag_position_buffer_y.append(y_f)

            # Calcular la media móvil de las últimas 300 posiciones
            if len(self.flag_position_buffer_x) == 300:  # Asegurarse de que el buffer esté lleno
                avg_x = sum(self.flag_position_buffer_x) / len(self.flag_position_buffer_x)
                avg_y = sum(self.flag_position_buffer_y) / len(self.flag_position_buffer_y)

                # Crear y almacenar el mensaje PoseStamped
                flag_pose = PoseStamped()
                flag_pose.header.stamp = self.get_clock().now().to_msg()
                flag_pose.header.frame_id = 'map'  # Ajusta el frame_id si es necesario
                flag_pose.pose.position.x = avg_x
                flag_pose.pose.position.y = avg_y
                flag_pose.pose.position.z = 0.0
                # Puedes ajustar la orientación si es necesario
                flag_pose.pose.orientation = orientation_q  # Opcional

                # Publicar la posición de la bandera
                self.flag_pub.publish(flag_pose)

                # Almacenar la última posición calculada de la bandera
                self.last_flag_pose = flag_pose

    def check_flag_timeout(self):
        # Verificar si ha pasado más de 1 segundo sin recibir mensajes de la bandera
        time_since_last_flag = (self.get_clock().now() - self.last_flag_msg_time).nanoseconds * 1e-9
        if time_since_last_flag > 1.0:
            self.flag_detected = False

    def quaternion_to_yaw(self, q):
        # Convertir cuaternión a ángulo yaw (orientación en el plano xy)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    flag_position_node = FlagPositionNode()
    try:
        rclpy.spin(flag_position_node)
    except KeyboardInterrupt:
        pass
    finally:
        flag_position_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
