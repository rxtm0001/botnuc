#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ament_index_python.packages import get_package_share_directory
import os

class BottleDetector(Node):

    def __init__(self):
        super().__init__('bottle_detector')

        # Publicador al tópico 'flag_detector' (String)
        self.flag_publisher = self.create_publisher(String, 'flag_detector', 10)

        # Publicador para la posición normalizada del centro del recuadro
        self.rect_center_pub = self.create_publisher(Float32, '/bottle_rect_center', 10)

        # Publicador al tópico '/flag_detected' (Bool)
        self.flag_detected_publisher = self.create_publisher(Bool, '/flag_detected', 10)

        # Publicador para la distancia a la bandera
        self.flag_distance_pub = self.create_publisher(Float32, '/flag_distance', 10)

        # Obtener la ruta al directorio 'models' dentro del paquete
        package_share_directory = get_package_share_directory('camera_detection')
        model_path = os.path.join(package_share_directory, 'models')

        # Rutas completas a los archivos del modelo
        config_path = os.path.join(model_path, 'yolov3-tiny.cfg')  # Usar un modelo más ligero
        weights_path = os.path.join(model_path, 'yolov3-tiny.weights')
        classes_path = os.path.join(model_path, 'coco.names')

        # Verificar que los archivos existen
        if not os.path.exists(config_path):
            self.get_logger().error(f"No se encontró el archivo de configuración: {config_path}")
        if not os.path.exists(weights_path):
            self.get_logger().error(f"No se encontró el archivo de pesos: {weights_path}")
        if not os.path.exists(classes_path):
            self.get_logger().error(f"No se encontró el archivo de clases: {classes_path}")

        # Cargar el modelo YOLO con rutas absolutas
        self.net = cv2.dnn.readNet(weights_path, config_path)
        self.classes = []
        with open(classes_path, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.output_layers = self.net.getUnconnectedOutLayersNames()

        # Usar CUDA si está disponible
        if cv2.cuda.getCudaEnabledDeviceCount() > 0:
            self.get_logger().info("Usando GPU para la detección.")
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        else:
            self.get_logger().info("Usando CPU para la detección.")

        # Suscriptores a los tópicos de imagen de color y profundidad
        self.bridge = CvBridge()
        self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        # Sincronizador de mensajes
        self.ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)

        cv2.namedWindow("Detección de Botella", cv2.WINDOW_AUTOSIZE)

    def image_callback(self, color_msg, depth_msg):
        try:
            # Convertir los mensajes de ROS2 a imágenes OpenCV
            color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

            # Procesar la imagen para detectar la botella
            self.detect_bottle(color_image, depth_image)
        except Exception as e:
            self.get_logger().error(f"Error en image_callback: {e}")
            import traceback
            traceback.print_exc()

    def detect_bottle(self, img, depth_image):
        # Inicializar la variable de detección
        flag_detected = False
        center_scaled = float('nan')  # Valor predeterminado si no se detecta la bandera

        # Preparar la imagen para YOLO
        height, width, channels = img.shape
        blob = cv2.dnn.blobFromImage(img, scalefactor=0.00392, size=(320, 320),
                                     mean=(0, 0, 0), swapRB=True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        # Analizar las detecciones
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and self.classes[class_id] == "bottle":
                    # Objeto detectado
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Aplicar supresión no máxima
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        if len(indexes) > 0:
            # Bandera detectada
            flag_detected = True

            for i in indexes.flatten():
                x, y, w, h = boxes[i]
                # Asegurar que las coordenadas están dentro de los límites
                x = max(0, min(x, img.shape[1] - 1))
                y = max(0, min(y, img.shape[0] - 1))
                w = max(1, min(w, img.shape[1] - x))
                h = max(1, min(h, img.shape[0] - y))

                # Dibujar rectángulo alrededor de la botella
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)

                # Calcular el centro del recuadro
                rect_center_x = x + w / 2
                img_width = img.shape[1]

                # Calcular la posición del centro del recuadro en una escala de 0 a 100
                center_scaled = (rect_center_x / img_width) * 100  # Valor entre 0 y 100

                # Mostrar el valor de rect_center en el log
                # self.get_logger().info(f'Bandera detectada. rect_center: {center_scaled}')

                # Extraer la región de profundidad correspondiente
                depth_region = depth_image[y:y + h, x:x + w]
                depth_region = depth_region.astype(np.float32)

                # Filtrar valores inválidos
                depth_region = depth_region[depth_region > 0]

                if depth_region.size > 0:
                    # Determinar la escala según el tipo de dato
                    if depth_image.dtype == np.uint16:
                        depth_scale = 0.001  # Convertir de milímetros a metros
                    else:
                        depth_scale = 1.0  # Asumir que ya está en metros

                    # Calcular la distancia
                    distance = np.median(depth_region) * depth_scale

                    # Aplicar factor de corrección si es necesario
                    factor_correccion = 1.0  # Ajusta este valor tras la calibración
                    distance *= factor_correccion

                    # Publicar el valor de distancia en /flag_distance
                    self.flag_distance_pub.publish(Float32(data=distance))

                    # Publicar el mensaje si la bandera es detectada
                    flag_message = String()
                    flag_message.data = f"Bandera detectada a {distance:.2f} metros"
                    self.flag_publisher.publish(flag_message)

                    # Mostrar la distancia en la imagen
                    cv2.putText(img, f"Distancia: {distance:.2f} m", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                else:
                    cv2.putText(img, "Distancia no disponible", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        else:
            # No se detectó la bandera
            flag_detected = False
            # self.get_logger().info('No se detectó la bandera.')

        # Publicar la posición escalada del centro del recuadro
        self.rect_center_pub.publish(Float32(data=center_scaled))

        # Publicar el estado de detección en '/flag_detected'
        self.flag_detected_publisher.publish(Bool(data=flag_detected))

        # Mostrar la imagen de color con la detección
        cv2.imshow("Detección de Botella", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    bottle_detector = BottleDetector()
    try:
        rclpy.spin(bottle_detector)
    except KeyboardInterrupt:
        bottle_detector.get_logger().info('Nodo detenido por el usuario.')
    finally:
        bottle_detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
