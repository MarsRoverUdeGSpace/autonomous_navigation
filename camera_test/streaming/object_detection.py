#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def callback_imagen(msg):
    # Convierte el mensaje ROS de imagen a un objeto OpenCV
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Convertir la imagen a espacio de color HSV
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Definir el rango de colores para el anaranjado en el espacio de color HSV
    naranja_bajo = np.array([5, 100, 100])    # H: 5
    naranja_alto = np.array([15, 255, 255])   # H: 15

    # Crear una máscara para segmentar la imagen por color
    mascara = cv2.inRange(image_hsv, naranja_bajo, naranja_alto)

    # Encontrar contornos en la máscara
    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filtrar los contornos por forma (por ejemplo, seleccionar solo los contornos cuadrados)
    mayor_contorno = None
    mayor_area = 0
    for contorno in contornos:
        area = cv2.contourArea(contorno)
        if area > mayor_area:
            mayor_area = area
            mayor_contorno = contorno

    # Encerrar el mayor contorno en un cuadrado
    if mayor_contorno is not None:
        x, y, w, h = cv2.boundingRect(mayor_contorno)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Mostrar la imagen con el mayor contorno encerrado en un cuadrado
    cv2.imshow("ZED Camera", image)
    cv2.waitKey(1)  # Espera un breve momento para procesar los eventos de ventana

def main():
    # Inicializa el nodo suscriptor
    rospy.init_node('hammer_detection_node', anonymous=True)

    # Suscríbete al tópico /camera/image_raw con el tipo de mensaje Image
    rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, callback_imagen)

    # Mantiene el nodo en ejecución
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
