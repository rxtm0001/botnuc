#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time
import threading
import psutil  # Importamos psutil para manejar procesos del sistema

# Importaciones necesarias para el cliente de acción
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus  # Importamos GoalStatus para los códigos de estado
from rcl_interfaces.msg import Log  # Importamos el mensaje Log para suscribirnos a /rosout

class CaptureFlagNode(Node):
    def __init__(self):
        super().__init__('capture_flag_node')
        self.get_logger().info("Nodo de captura la bandera iniciado.")

        # Cliente de acción para la navegación
        self.navigation_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Suscriptor a la posición de la bandera en /flag_position
        self.flag_position_subscription = self.create_subscription(
            PoseStamped,
            '/flag_position',
            self.flag_position_callback,
            10
        )

        # Suscriptor al tópico de odometría para obtener la posición del robot
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Suscriptor al tópico /rosout para escuchar mensajes de registro
        self.rosout_subscription = self.create_subscription(
            Log,
            '/rosout',
            self.rosout_callback,
            10
        )

        # Variables para almacenar posiciones
        self.flag_position = None
        self.initial_position = None

        # Banderas de estado
        self.flag_detected = False
        self.initial_position_saved = False

        # Estado del juego
        self.game_state = 'WAITING_FOR_FLAG'  # Otros estados: 'MENU', 'CAPTURING_FLAG', 'RETURNING_HOME'

        # Variable para indicar si el cliente de acción está listo
        self.action_client_ready = False

        # Iniciar un hilo para esperar a que el cliente de acción esté listo
        threading.Thread(target=self.wait_for_action_server).start()

    def wait_for_action_server(self):
        self.get_logger().info("Esperando a que el servidor de acción de navegación esté disponible...")
        self.navigation_action_client.wait_for_server()
        self.get_logger().info("Servidor de acción de navegación disponible.")
        self.action_client_ready = True

    def rosout_callback(self, msg):
        if msg.name == 'approach_flag_controller' and '¡Bandera alcanzada!' in msg.msg:
            self.get_logger().info("Recibido '¡Bandera alcanzada!' desde approach_flag_controller")
            # Proceder a cerrar el nodo approach_flag_controller
            self.shutdown_approach_flag_controller()

    def shutdown_approach_flag_controller(self):
        self.get_logger().info("Intentando cerrar approach_flag_controller")
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if 'approach_flag_controller' in proc.info['cmdline']:
                    self.get_logger().info(f"Terminando proceso {proc.info['pid']} ({proc.info['name']})")
                    proc.kill()
                    return
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        self.get_logger().error("No se pudo encontrar el proceso de approach_flag_controller para cerrar.")

    def odom_callback(self, msg):
        if not self.initial_position_saved:
            # Guardar la posición inicial del robot
            self.initial_position = msg.pose.pose
            self.initial_position_saved = True
            self.get_logger().info("Posición inicial guardada: ({:.2f}, {:.2f})".format(
                self.initial_position.position.x, self.initial_position.position.y))

    def flag_position_callback(self, msg):
        if not self.flag_detected:
            # Guardar la posición de la bandera y establecer la bandera como detectada
            self.flag_position = msg
            self.flag_detected = True
            self.get_logger().info("¡La bandera ha sido detectada!")
            # Mostrar el menú de opciones al usuario
            self.show_menu()

    def show_menu(self):
        print("\n--- Menú de Opciones ---")
        print("1. Ir a buscar la bandera")
        print("2. Regresar a la posición inicial")
        print("3. Cancelar y salir")

        option = input("Seleccione una opción: ")

        if option == "1":
            self.game_state = 'CAPTURING_FLAG'
            self.capture_flag()
        elif option == "2":
            self.game_state = 'RETURNING_HOME'
            self.return_to_initial_position()
        elif option == "3":
            self.shutdown_robot()
        else:
            print("Opción no válida, por favor intente de nuevo.")
            self.show_menu()

    def capture_flag(self):
        self.get_logger().info("Iniciando captura de la bandera...")
        if self.action_client_ready:
            # Enviar objetivo de navegación a la posición de la bandera
            self.send_navigation_goal(self.flag_position)
        else:
            self.get_logger().error("El cliente de acción no está listo. No se puede navegar.")

    def return_to_initial_position(self):
        self.get_logger().info("Regresando a la posición inicial...")
        if self.action_client_ready:
            # Crear un PoseStamped para la posición inicial
            initial_pose_stamped = PoseStamped()
            initial_pose_stamped.header.frame_id = 'map'  # Asegúrate de que este frame_id es correcto
            initial_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            initial_pose_stamped.pose = self.initial_position

            # Enviar objetivo de navegación a la posición inicial
            self.send_navigation_goal(initial_pose_stamped)
        else:
            self.get_logger().error("El cliente de acción no está listo. No se puede navegar.")

    def send_navigation_goal(self, pose_stamped):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.get_logger().info(f"Enviando objetivo de navegación a ({pose_stamped.pose.position.x:.2f}, {pose_stamped.pose.position.y:.2f})")

        # Enviar el objetivo de navegación y asignar un callback para el resultado
        self._send_goal_future = self.navigation_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.navigation_goal_response_callback)

    def navigation_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Objetivo de navegación rechazado.')
            return

        self.get_logger().info('Objetivo de navegación aceptado.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Puedes agregar aquí lógica para manejar el feedback si es necesario
        self.get_logger().info(f"Navegando... Distancia restante: {feedback.distance_remaining:.2f} metros")

    def navigation_result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Objetivo de navegación alcanzado con éxito.')
            # Dependiendo del estado actual, decidir qué hacer después
            if self.game_state == 'CAPTURING_FLAG':
                self.get_logger().info("¡Bandera capturada!")
                self.show_menu_after_capture()
            elif self.game_state == 'RETURNING_HOME':
                self.get_logger().info("El robot ha regresado a la posición inicial.")
                self.shutdown_robot()
        elif status == GoalStatus.STATUS_ABORTED or status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error('La navegación fue interrumpida o falló.')
        else:
            self.get_logger().info(f"Estado de navegación desconocido: {status}")

    def show_menu_after_capture(self):
        print("\n--- Menú de Opciones ---")
        print("1. Regresar a la posición inicial")
        print("2. Finalizar el juego")

        option = input("Seleccione una opción: ")

        if option == "1":
            self.game_state = 'RETURNING_HOME'
            self.return_to_initial_position()
        elif option == "2":
            self.shutdown_robot()
        else:
            print("Opción no válida, por favor intente de nuevo.")
            self.show_menu_after_capture()

    def shutdown_robot(self):
        self.get_logger().info("Finalizando el juego y deteniendo el robot.")
        self.stop_robot()
        rclpy.shutdown()

    def stop_robot(self):
        # Función para detener el robot en cualquier momento
        # Publicar un Twist vacío en cmd_vel para detener el robot
        pass  # Implementa esto si es necesario

def main(args=None):
    rclpy.init(args=args)
    capture_flag_node = CaptureFlagNode()

    try:
        rclpy.spin(capture_flag_node)
    except KeyboardInterrupt:
        capture_flag_node.get_logger().info("Nodo de captura la bandera interrumpido por el usuario.")
        capture_flag_node.stop_robot()
    finally:
        capture_flag_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
