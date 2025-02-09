#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class SearchFlagNode(Node):
    def __init__(self):
        super().__init__('search_flag_node')
        self.get_logger().info("Nodo de búsqueda de la bandera iniciado.")

        # Publicador al tópico /cmd_vel para mover el robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscriptor al tópico de detección de la bandera
        self.flag_detected_subscription = self.create_subscription(
            Bool,
            '/flag_detected',
            self.flag_detected_callback,
            10
        )

        self.flag_detected = False

        # Crear un timer para publicar el comando de rotación cada cierto tiempo
        timer_period = 0.1  # 100 ms
        self.timer = self.create_timer(timer_period, self.publish_rotation_command)

    def publish_rotation_command(self):
        if not self.flag_detected:
            twist = Twist()
            # Establecer una velocidad angular en z para rotar
            twist.angular.z = 0.2  # Velocidad angular en rad/s, ajustar según sea necesario
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info("Rotando para buscar la bandera...")
        else:
            # Detener el robot
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)
            self.get_logger().info("Bandera detectada, deteniendo la búsqueda.")
            # Cerrar el nodo
            self.get_logger().info("Cerrando nodo de búsqueda de la bandera.")
            self.destroy_node()
            rclpy.shutdown()

    def flag_detected_callback(self, msg):
        if msg.data:
            self.flag_detected = True

def main(args=None):
    rclpy.init(args=args)
    search_flag_node = SearchFlagNode()

    rclpy.spin(search_flag_node)

if __name__ == '__main__':
    main()
