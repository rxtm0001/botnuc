#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import threading
import subprocess

# Importaciones necesarias para el cliente de acción
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool, String  # Importamos String para el comando del electroimán

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

        # Suscriptor al tópico de detección de la bandera
        self.flag_detected_subscription = self.create_subscription(
            Bool,
            '/flag_detected',
            self.flag_detected_callback,
            10
        )

        # Suscriptor al tópico /flag_ready
        self.flag_ready_subscription = self.create_subscription(
            Bool,
            '/flag_ready',
            self.flag_ready_callback,
            10
        )

        # Publicador al tópico electromagnet_command
        self.electromagnet_publisher = self.create_publisher(
            String,
            'electromagnet_command',
            10
        )

        # Variables para almacenar posiciones y estados
        self.flag_position = None
        self.initial_position = None
        self.flag_detected = False
        self.flag_confirmed = False  # Indica si la bandera fue confirmada después de llegar al objetivo
        self.initial_position_saved = False
        self.should_shutdown = False  # Bandera para indicar que el nodo debe cerrarse

        #Intentos por si nav2 falla
        self.navigation_retry_count = 0
        self.navigation_max_retries = 3
        self.last_sent_goal_pose_stamped = None


        # Bandera para mostrar menús
        self.show_menu_flag = False
        self.show_menu_after_capture_flag = False

        # Bandera para mostrar el mensaje de espera
        self.waiting_for_flag_message_shown = False

        # Estado del juego
        self.game_state = 'WAITING_FOR_FLAG'  # Otros estados: 'MENU', 'NAVIGATING_TO_FLAG', 'CONFIRMING_FLAG', 'RETURNING_HOME'

        # Variable para indicar si el cliente de acción está listo
        self.action_client_ready = False

        # Variable para manejar el proceso del nodo ApproachFlagController
        self.approach_flag_process = None

        # Bandera para evitar iniciar múltiples veces el nodo ApproachFlagController
        self.approach_flag_node_started = False

        # Lock para sincronizar el acceso a approach_flag_node_started
        self.lock = threading.Lock()  ### Añadido

        # Iniciar un hilo para esperar a que el servidor de acciones esté listo
        threading.Thread(target=self.wait_for_action_server).start()

    def wait_for_action_server(self):
        self.get_logger().info("Esperando a que el servidor de acción de navegación esté disponible...")
        self.navigation_action_client.wait_for_server()
        self.get_logger().info("Servidor de acción de navegación disponible.")
        self.action_client_ready = True

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
            # Señalar que se debe mostrar el menú
            self.show_menu_flag = True
            # Resetear la variable para futuros usos
            self.waiting_for_flag_message_shown = False

    def flag_detected_callback(self, msg):
        # Callback para actualizar el estado de detección de la bandera
        self.flag_confirmed = msg.data

        if self.game_state == 'CONFIRMING_FLAG':
            with self.lock:  ### Añadido
                if not self.approach_flag_node_started:
                    self.approach_flag_node_started = True  ### Mover aquí
                    if self.flag_confirmed:
                        self.get_logger().info("¡Confirmación: la bandera está visible en la ubicación!")
                    else:
                        self.get_logger().info("La bandera no está visible en la ubicación.")
                    # Iniciar el nodo para acercarse y agarrar la bandera
                    self.start_approach_flag_node()

    def flag_ready_callback(self, msg):
        if msg.data:
            self.get_logger().info("¡Bandera lista para ser agarrada!")
            # Encender el electroimán
            self.electromagnet_publisher.publish(String(data='ON'))
            self.get_logger().info("Electroimán encendido.")
            # Proceder a regresar a la posición inicial
            self.game_state = 'RETURNING_HOME'
            self.return_to_initial_position()

    def show_menu(self):
        print("\n--- Menú de Opciones ---")
        print("1. Ir a buscar la bandera")
        print("2. Regresar a la posición inicial")
        print("3. Cancelar y salir")

        option = input("Seleccione una opción: ")

        if option == "1":
            self.game_state = 'NAVIGATING_TO_FLAG'
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
        self.navigation_retry_count = 0  # Resetea el contador de reintentos
        self.last_sent_goal_pose_stamped = pose_stamped  # Guarda el último objetivo enviado

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
            if future.exception() is not None:
                self.get_logger().error(f'Error al obtener el resultado de la navegación: {future.exception()}')
                return

            status = future.result().status
            self.get_logger().info(f"Código de estado recibido: {status}")

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Objetivo de navegación alcanzado con éxito.')
                self.navigation_retry_count = 0  # Resetea el contador en caso de éxito

                # Dependiendo del estado actual, decidir qué hacer después
                if self.game_state == 'NAVIGATING_TO_FLAG':
                    self.game_state = 'CONFIRMING_FLAG'
                    self.confirm_flag_at_location()
                elif self.game_state == 'RETURNING_HOME':
                    self.get_logger().info("El robot ha regresado a la posición inicial.")
                    # Apagar el electroimán al regresar a casa
                    self.electromagnet_publisher.publish(String(data='OFF'))
                    self.get_logger().info("Electroimán apagado.")
                    # Mostrar el menú nuevamente
                    self.game_state = 'MENU'
                    self.show_menu_flag = True

            elif status == GoalStatus.STATUS_ABORTED or status == GoalStatus.STATUS_CANCELED:
                self.get_logger().error('La navegación fue interrumpida o falló.')

                if self.navigation_retry_count < self.navigation_max_retries:
                    self.navigation_retry_count += 1
                    self.get_logger().info(f'Reintentando enviar el objetivo de navegación... Intento {self.navigation_retry_count}')
                    # Reenviar el último objetivo guardado
                    self.send_navigation_goal(self.last_sent_goal_pose_stamped)
                else:
                    self.get_logger().error('Se alcanzó el número máximo de reintentos de navegación.')
                    # Aquí puedes decidir qué hacer: mostrar menú, finalizar, etc.
                    # Por ejemplo, podrías mostrar el menú nuevamente
                    self.game_state = 'MENU'
                    self.show_menu_flag = True

            else:
                self.get_logger().info(f"Estado de navegación desconocido: {status}")


    def confirm_flag_at_location(self):
        self.get_logger().info("Confirmando si la bandera está visible en la ubicación actual...")
        # La detección se maneja en el callback 'flag_detected_callback'

    def start_approach_flag_node(self):
        # Ya no encendemos el electroimán aquí, se hará después de recibir '/flag_ready'
        self.get_logger().info("Iniciando nodo para acercarse y agarrar la bandera...")
        command = "ros2 run behavior_tree ApproachFlagController.py"
        self.approach_flag_process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f"source ~/linorobot2_ws/install/setup.bash; {command}; exec bash"])
        # No esperamos aquí, el proceso se manejará asíncronamente

    def stop_process(self, process):
        if process is not None:
            self.get_logger().info(f"Deteniendo proceso con PID {process.pid}")
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().warning(f"El proceso con PID {process.pid} no se detuvo a tiempo. Forzando cierre.")
                process.kill()

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
        self.should_shutdown = True  # Indicamos que el nodo debe cerrarse

    def stop_robot(self):
        # Implementa aquí la lógica para detener el robot si es necesario
        # También detenemos cualquier proceso que esté en ejecución
        with self.lock:  ### Añadido
            if self.approach_flag_process is not None:
                self.stop_process(self.approach_flag_process)
                self.approach_flag_process = None
                self.approach_flag_node_started = False

def main(args=None):
    rclpy.init(args=args)
    capture_flag_node = CaptureFlagNode()

    try:
        while rclpy.ok() and not capture_flag_node.should_shutdown:
            rclpy.spin_once(capture_flag_node, timeout_sec=0.1)

            # Si la bandera aún no ha sido detectada, mostramos un mensaje
            if not capture_flag_node.flag_detected and not capture_flag_node.waiting_for_flag_message_shown:
                capture_flag_node.get_logger().info("Esperando que se detecte la bandera...")
                capture_flag_node.waiting_for_flag_message_shown = True

            if capture_flag_node.show_menu_flag:
                capture_flag_node.show_menu_flag = False
                capture_flag_node.show_menu()
            if capture_flag_node.show_menu_after_capture_flag:
                capture_flag_node.show_menu_after_capture_flag = False
                capture_flag_node.show_menu_after_capture()
    except KeyboardInterrupt:
        capture_flag_node.get_logger().info("Nodo de captura la bandera interrumpido por el usuario.")
        capture_flag_node.stop_robot()
    finally:
        capture_flag_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
