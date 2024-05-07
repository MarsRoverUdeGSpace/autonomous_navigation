#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
from sensor_msgs.msg import Image
import camera_functions as cam 
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
import rospy
import numpy as np
from PIL import Image as PILImage
from io import BytesIO
def imgmsg_to_cv2(img_msg, dtype=np.uint8):
    # it should be possible to determine dtype from img_msg.encoding but there is many different cases to take into account
    # original function args: imgmsg_to_cv2(img_msg, desired_encoding = "passthrough")
    return np.frombuffer(img_msg.data, dtype=dtype).reshape(img_msg.height, img_msg.width, -1)

def callback_imagen(msg):
    global pub
    data_image = imgmsg_to_cv2(msg)
    #image = cv2.cvtColor(data_image, cv2.COLOR_BGR2RGB)
    image = cv2.resize(data_image, (0, 0), fx=2, fy=2)
    # Procesa la imagen recibida (por ejemplo, muestra la imagen)
    cv2.imshow("ZED Camera", image)
    cv2.waitKey(1)  # Espera un breve momento para procesar los eventos de ventana
    # try:
    # Convertir los datos de la imagen a un objeto PIL Image

    # image_data = BytesIO(msg.data)
    # pil_image = PILImage.open(image_data)

    # Convertir la imagen PIL a una matriz numpy
    # image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)

    # # Convertir la imagen a escala de grises
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # # Definir el diccionario de marcadores
    # aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

    # # Definir los parámetros del detector
    # parameters = cv2.aruco.DetectorParameters_create()

    # # Definimos los parámetros de la cámara
    # camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])
    # distortion = np.array([0, 0, 0, 0])

    # # Detectar códigos ArUco en la imagen
    # corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # # Leer la imagen
    # frame = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)

    # # Publicar la información de ArUco si se encuentran códigos
    # if len(corners) > 0:
    #     for i in range(len(ids)):
    #         corners_first_tag = corners[i]
    #         avg_x = int(np.mean(corners_first_tag[:, :, 0]))
    #         avg_y = int(np.mean(corners_first_tag[:, :, 1]))

    #         # Publicar información de ArUco
    #         aruco_msg = Float32MultiArray()
    #         aruco_msg.data = [ids[i], avg_x, avg_y]
    #         pub.publish(aruco_msg)

    # # Mostrar la imagen
    # cv2.imshow("ZED Camera", frame)
    # cv2.waitKey(1)

    # except Exception as e:
    #     rospy.logerr(e)
def main():
    global pub
    # Inicializa el nodo suscriptor
    #rospy.init_node('aruco_detection_node', anonymous=True)
    rospy.init_node('aruco_detection_node')

    # Subscriptor al tópico /zed2i/zed_node/rgb/image_rect_color con el tipo de mensaje Image
    rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, callback_imagen) 
    rospy.wait_for_message("/zed2i/zed_node/rgb/image_rect_color", Image)  # Espera a que llegue el primer mensaje
    
    pub = rospy.Publisher("/Aruco",Float32MultiArray,queue_size=10)
    # Mantiene el nodo en ejecución
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
