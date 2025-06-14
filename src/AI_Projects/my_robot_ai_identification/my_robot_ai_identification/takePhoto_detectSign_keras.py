#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from tensorflow.keras.models import load_model
import time
from datetime import datetime
import os

class KerasImageClassifier(Node):
    def __init__(self):
        super().__init__('keras_detector_node')
        # Paths relativos al script
        script_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = os.path.join(script_dir, "../models/keras_model.h5")
        labels_path = os.path.join(script_dir, "../models/labels.txt")

        # Cargar modelo y etiquetas
        self.model = load_model(model_path)
        self.labels = self.load_labels(labels_path)
        self.input_shape = self.model.input_shape[1:3]

        # Imagen
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            10  # QoS history depth
        )
        self.class_pub = self.create_publisher(String, "/predicted_class", 10)

        # Capturas
        self.capture_enabled = True
        self.capture_dir = os.path.expanduser("~/ROS2_LIMO_ws/src/AI_Projects/my_robot_ai_identification/photos")
        os.makedirs(self.capture_dir, exist_ok=True)
        self.create_class_dirs()  # Crear carpetas por clase

        self.last_capture_time = self.get_clock().now().nanoseconds * 1e-9
        self.capture_interval = 1.0  # segundos

        # Control de captura por topic
        self.capture_toggle_sub = self.create_subscription(
            Bool,
            "/capture_toggle",
            self.toggle_callback,
            10
        )

        self.get_logger().info("Nodo keras_detector activo. Esperando imágenes...")

    def load_labels(self, path):
        with open(path, 'r') as f:
            lines = f.readlines()
            return [line.strip().split(' ', 1)[1] for line in lines]

    def create_class_dirs(self):
        for class_name in self.labels:
            class_path = os.path.join(self.capture_dir, class_name)
            os.makedirs(class_path, exist_ok=True)

    def toggle_callback(self, msg):
        self.capture_enabled = msg.data
        state = "ACTIVADA" if self.capture_enabled else "DESACTIVADA"
        self.get_logger().info(f"Captura automática {state}")

    def image_callback(self, msg):
        try:
            # Convertir imagen
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            resized = cv2.resize(cv_image, self.input_shape)
            img = resized.astype(np.float32) / 255.0
            img = np.expand_dims(img, axis=0)

            # Predicción
            predictions = self.model.predict(img)
            class_index = np.argmax(predictions)
            class_name = self.labels[class_index]
            self.get_logger().info(f"Detectado: {class_name}")
            string_msg = String()
            string_msg.data = class_name
            self.class_pub.publish(string_msg)

            # Guardar imagen si está activado
            if self.capture_enabled:
                current_time = self.get_clock().now().nanoseconds * 1e-9
                if current_time - self.last_capture_time >= self.capture_interval:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = f"{class_name}_{timestamp}.jpg"
                    class_folder = os.path.join(self.capture_dir, class_name)
                    filepath = os.path.join(class_folder, filename)
                    cv2.imwrite(filepath, cv_image)
                    self.get_logger().info(f"Imagen guardada: {filepath}")
                    self.last_capture_time = current_time

        except CvBridgeError as e:
            self.get_logger().error(f"Error de CvBridge: {e}")
        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = KerasImageClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()