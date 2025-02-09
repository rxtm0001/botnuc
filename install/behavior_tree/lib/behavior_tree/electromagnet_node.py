#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class ElectromagnetNode(Node):
    def __init__(self):
        super().__init__('electromagnet_node')

        # Configura el puerto serial (cambia '/dev/ttyUSB0' si es necesario)
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
            self.get_logger().info('Puerto serial conectado correctamente.')
        except serial.SerialException as e:
            self.get_logger().error(f'Error al conectar el puerto serial: {e}')
            self.destroy_node()
            return

        # Variable para rastrear el último comando enviado
        self.last_command_sent = None

        # Suscriptor al tópico 'electromagnet_command'
        self.subscription = self.create_subscription(
            String,
            'electromagnet_command',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        command = msg.data.strip().upper()
        if command in ['ON', 'OFF']:
            if command != self.last_command_sent:
                self.serial_port.write((command + '\n').encode())
                self.get_logger().info(f'Enviando comando al Arduino: {command}')
                self.last_command_sent = command
            else:
                self.get_logger().info(f'Comando "{command}" ya fue enviado anteriormente. No se envía de nuevo.')
        else:
            self.get_logger().warning(f'Comando inválido recibido: {command}')

    def destroy_node(self):
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ElectromagnetNode()
    if rclpy.ok():
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
