#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import os
import time
import psutil

class NodeSupervisor(Node):
    def __init__(self):
        super().__init__('node_supervisor')
        self.processes = []  # Lista para almacenar los procesos iniciados

        # Iniciar inmediatamente sin solicitar confirmación
        self.start_robot()
        print("Esperando 5 segundos antes de mostrar las opciones...")
        time.sleep(5)  # Delay de 5 segundos antes de mostrar el menú
        self.show_menu()  # Mostramos el menú después del delay

    def start_robot(self):
        self.get_logger().info("Iniciando nodos...")

        # Inicia cada comando en una nueva terminal y guarda el proceso
        process1 = subprocess.Popen(['gnome-terminal', '--', 'ros2', 'launch', 'linorobot2_bringup', 'bringup.launch.py'])
        time.sleep(5)  # Delay de 5 segundos para que cargue el robot
        process2 = subprocess.Popen(['gnome-terminal', '--', 'ros2', 'launch', 'linorobot2_bringup', 'joy_teleop.launch.py'])
        time.sleep(2)  # Delay de 2 segundos para que cargue el robot
        process3 = subprocess.Popen(['gnome-terminal', '--', 'ros2', 'launch', 'linorobot2_navigation', 'slam.launch.py', 'rviz:=true'])
        time.sleep(5)  # Delay de 5 segundos para que cargue el mapa
        process4 = subprocess.Popen(['gnome-terminal', '--', 'ros2', 'launch', 'camera_detection', 'camera_detection_launch.py'])
        time.sleep(5)  # Delay de 5 segundos para que cargue la cámara
        process5 = subprocess.Popen(['gnome-terminal', '--', 'ros2', 'run', 'behavior_tree', 'electromagnet_node.py'])
        time.sleep(5)  # Delay de 5 segundos para el nodo del electroimán

        # Almacenar los procesos en una lista para cerrarlos después
        self.processes.extend([process1, process2, process3, process4, process5])

        self.get_logger().info("El robot se ha iniciado correctamente")

    def show_menu(self):
        try:
            while True:
                print("\n--- Opciones del Sistema ---")
                print("1. Guardar Mapa")
                print("2. Abrir mapa")
                print("3. Programa bandera")
                print("5. Salir")
                
                option = input("Seleccione una opción: ")

                if option == "1":
                    self.save_map()
                elif option == "2":
                    self.open_map()
                elif option == "3":
                    self.capture_flag()
                elif option == "5":
                    print("Saliendo del sistema...")
                    self.shutdown_robot()
                    break
                else:
                    print("Opción no válida, por favor intente de nuevo.")
                        
        except KeyboardInterrupt:
            print("\nInterrupción detectada. Cerrando el sistema...")
            self.shutdown_robot()

    def save_map(self):
        print("\n--- Guardar Mapa ---")
        map_name = input("Ingrese el nombre del mapa: ")
        
        # Comando para guardar el mapa
        try:
            os.chdir("/home/botnuc/linorobot2_ws/src/linorobot2/linorobot2_navigation/maps")
            save_command = f"ros2 run nav2_map_server map_saver_cli -f {map_name} --ros-args -p save_map_timeout:=10000.0"
            subprocess.run(save_command, shell=True, check=True)
            print(f"El mapa '{map_name}' se ha guardado correctamente.")
        except subprocess.CalledProcessError as e:
            print(f"Error al guardar el mapa: {e}")
        except FileNotFoundError as e:
            print(f"Ruta no encontrada: {e}")
        finally:
            os.chdir("/home/botnuc/linorobot2_ws")

    def open_map(self):
        print("\n--- Abrir Mapa ---")

        maps_path = "/home/botnuc/linorobot2_ws/src/linorobot2/linorobot2_navigation/maps"

        try:
            map_files = [f for f in os.listdir(maps_path) if f.endswith('.yaml')]
            if not map_files:
                print("No se encontraron mapas disponibles.")
                return

            print("Mapas disponibles:")
            for i, map_file in enumerate(map_files):
                print(f"{i + 1}. {map_file}")

            selection = input("Seleccione un mapa por número o ingrese el nombre del archivo (con .yaml): ")
            if selection.isdigit() and 1 <= int(selection) <= len(map_files):
                map_name = map_files[int(selection) - 1]
            elif selection in map_files:
                map_name = selection
            else:
                print("Selección no válida.")
                return

            map_path = os.path.join(maps_path, map_name)
            open_command = f"ros2 launch linorobot2_navigation navigation.launch.py map:={map_path} rviz:=true"
            process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', open_command])
            self.processes.append(process)
            print(f"El mapa '{map_name}' se está abriendo.")
        except Exception as e:
            print(f"Error al abrir el mapa: {e}")

    def capture_flag(self):
        print("\n--- Iniciando Programa Captura la Bandera ---")
        process = subprocess.Popen(['gnome-terminal', '--', 'ros2', 'run', 'behavior_tree', 'capture_flag_node.py'])
        self.processes.append(process)

    def shutdown_robot(self):
        print("Finalizando nodos...")

        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            if any(target in proc.info['cmdline'] for target in [
                'bringup.launch.py', 'slam.launch.py', 'joy_teleop.launch.py',
                'camera_detection_launch.py', 'navigation.launch.py', 'capture_flag_node.py', 'electromagnet_node.py']):
                try:
                    print(f"Matando proceso {proc.info['pid']} relacionado con {proc.info['cmdline']}")
                    proc.terminate()
                    proc.wait(timeout=5)
                except psutil.NoSuchProcess:
                    continue
                except psutil.TimeoutExpired:
                    print(f"Proceso {proc.info['pid']} no se cerró a tiempo. Forzando cierre con kill.")
                    proc.kill()

        print("Todos los nodos han sido cerrados correctamente.")

def main(args=None):
    rclpy.init(args=args)
    node_supervisor = NodeSupervisor()

    try:
        rclpy.spin(node_supervisor)
    except KeyboardInterrupt:
        node_supervisor.get_logger().info("Interrupción detectada. Cerrando nodos...")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
