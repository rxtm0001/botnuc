#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
import math

class ApproachFlagController(Node):
    def __init__(self):
        super().__init__('approach_flag_controller')

        # Suscribirse a /flag_detected
        self.flag_detected_sub = self.create_subscription(
            Bool,
            '/flag_detected',
            self.flag_detected_callback,
            qos_profile=10  ### Añadido qos_profile
        )

        # Suscribirse a /bottle_rect_center
        self.rect_center_sub = self.create_subscription(
            Float32,
            '/bottle_rect_center',
            self.rect_center_callback,
            qos_profile=10  ### Añadido qos_profile
        )

        # Suscribirse a /flag_distance
        self.flag_distance_sub = self.create_subscription(
            Float32,
            '/flag_distance',
            self.flag_distance_callback,
            qos_profile=10  ### Añadido qos_profile
        )

        # Suscribirse a /odom
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=10  ### Añadido qos_profile
        )

        # Publicador para /cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publicador para /flag_ready ###
        self.flag_ready_pub = self.create_publisher(
            Bool,
            '/flag_ready',
            10
        )  ###

        # Almacenar la posición del robot
        self.robot_pose = None
        self.rect_center = None  # Inicializado como None
        self.flag_distance = None  # Inicializado como None
        self.flag_detected = False  # Bandera detectada
        self.rotation_count = 0  ### Contador de rotaciones completas
        self.prev_yaw = None  ### Ángulo previo para calcular la rotación
        self.yaw_accumulated = 0.0  # Acumulador de cambio de yaw para detectar rotaciones

        # Parámetros de control
        self.linear_speed = 0.5  # Velocidad lineal máxima (m/s)
        self.angular_speed = 0.2  # Velocidad angular máxima (rad/s)
        self.center_threshold = 20  # Umbral para considerar la bandera centrada (en unidades de error)
        self.min_distance_to_flag = 0.15  # Distancia mínima al objetivo (en metros)

        # Crear un temporizador para llamar a navigate_to_flag periódicamente
        self.timer = self.create_timer(0.1, self.navigate_to_flag)  # Cada 0.1 segundos

    def flag_detected_callback(self, msg):
        self.flag_detected = msg.data

    def rect_center_callback(self, msg):
        alpha = 0.3  # Factor de suavizado
        if math.isnan(msg.data):
            self.rect_center = None
        else:
            if self.rect_center is None:
                self.rect_center = msg.data
            else:
                self.rect_center = alpha * msg.data + (1 - alpha) * self.rect_center

    def flag_distance_callback(self, msg):
        self.flag_distance = msg.data

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def navigate_to_flag(self):
        if self.robot_pose is None:
            return  # Necesitamos la posición del robot para proceder

        twist_msg = Twist()
        yaw = self.get_yaw_from_quaternion(self.robot_pose.orientation)  ### Obtener la orientación actual del robot

        if self.flag_detected and self.rect_center is not None and self.flag_distance is not None:
            # Calcular el error desde el centro (50 es el centro)
            center_error = self.rect_center - 50

            # Control proporcional para el centrado de la imagen
            Kp_image_centering = 1.5
            image_centering_adjustment = Kp_image_centering * (-center_error / 50)
            twist_msg.angular.z = max(min(image_centering_adjustment, self.angular_speed), -self.angular_speed)

            # Control de distancia: avanzar hasta que estemos cerca de la bandera
            if abs(center_error) < self.center_threshold and self.flag_distance > self.min_distance_to_flag:
                twist_msg.linear.x = self.linear_speed
                self.get_logger().info(f'Bandera centrada. Avanzando hacia ella a {self.flag_distance:.2f} metros...')
            elif self.flag_distance <= self.min_distance_to_flag:
                twist_msg.linear.x = 0.0
                self.get_logger().info('Distancia mínima alcanzada. Deteniéndose.')
            else:
                twist_msg.linear.x = 0.0
                self.get_logger().info('Ajustando orientación para centrar la bandera...')

            # Información de depuración
            self.get_logger().info(f'rect_center={self.rect_center:.2f}, center_error={center_error:.2f}, '
                                   f'angular_z={twist_msg.angular.z:.2f}, distance={self.flag_distance:.2f}')
            self.rotation_count = 0  ### Reiniciar el contador de rotaciones si la bandera está detectada
        else:
            # Si la bandera no está detectada o no tenemos información de centrado
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -self.angular_speed  ### Cambiar la velocidad angular para realizar una rotación completa
            self.get_logger().info('Bandera no detectada o sin información de centrado. Girando para buscarla...')

            # Detectar una rotación completa de 360 grados
            if self.prev_yaw is not None:
                delta_yaw = self.normalize_angle(yaw - self.prev_yaw)
                self.yaw_accumulated += delta_yaw
                self.get_logger().info(f'Acumulando delta_yaw: {delta_yaw:.2f} rad, yaw_accumulated: {self.yaw_accumulated:.2f} rad')

                # Si hemos acumulado 2*pi radianes (aproximadamente 6.38), incrementamos el contador de rotaciones
                if abs(self.yaw_accumulated) >= 2 * math.pi:
                    self.rotation_count += 1
                    self.yaw_accumulated = 0.0  # Reiniciar el acumulador
                    self.get_logger().info(f'Rotación completa detectada. Conteo de rotaciones: {self.rotation_count}')

            self.prev_yaw = yaw  # Actualizar el valor previo de yaw

            if self.rotation_count >= 1:  ### Verificar si se han completado dos rotaciones
                self.get_logger().info('Bandera lista para ser agarrada. Deteniendo rotación.')
                twist_msg.angular.z = 0.0  ### Detener la rotación

                # Publicar mensaje de que la bandera está lista para ser agarrada
                self.flag_ready_pub.publish(Bool(data=True))  ###
                
                # Destruir el nodo para detener completamente su ejecución ###
                self.get_logger().info('Nodo ApproachFlagController finalizado después de completar la tarea.')
                self.destroy_node()  ###
                return  ###

        # Almacenar el ángulo actual para el cálculo de rotación en la siguiente iteración
        self.prev_yaw = yaw  ###

        # Publicar comandos de velocidad
        self.cmd_vel_pub.publish(twist_msg)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = ApproachFlagController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
