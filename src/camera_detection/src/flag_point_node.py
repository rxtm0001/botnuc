#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped

class FlagPointNode(Node):

    def __init__(self):
        super().__init__('flag_point_node')

        # Suscribirse a /flag_position
        self.flag_pose_sub = self.create_subscription(
            PoseStamped,
            '/flag_position',
            self.flag_pose_callback,
            10
        )

        # Publicador para /flag_point
        self.flag_point_pub = self.create_publisher(
            PointStamped,
            '/flag_point',
            10
        )

    def flag_pose_callback(self, msg):
        # Crear un mensaje PointStamped
        point_msg = PointStamped()
        point_msg.header = msg.header  # Copiar el header del PoseStamped
        point_msg.point = msg.pose.position  # Copiar la posición

        # Publicar el mensaje
        self.flag_point_pub.publish(point_msg)
        # self.get_logger().info(f'Posición de la bandera publicada en /flag_point: x={point_msg.point.x}, y={point_msg.point.y}')

def main(args=None):
    rclpy.init(args=args)
    flag_point_node = FlagPointNode()
    try:
        rclpy.spin(flag_point_node)
    except KeyboardInterrupt:
        pass
    finally:
        flag_point_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()