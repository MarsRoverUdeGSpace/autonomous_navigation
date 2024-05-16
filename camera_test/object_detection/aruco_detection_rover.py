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
    # print(msg.encoding)
    image = imgmsg_to_cv2(msg)
    # image2 = image[:, :, :3]  # Elimina el cuarto canal (canal alfa)
    # image = cv2.merge(image)

    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    # Procesa la imagen recibida (por ejemplo, muestra la imagen)
    # cv2.imshow("ZED Camera", image)
    # cv2.waitKey(1)  # Espera un breve momento para procesar los eventos de ventana
    
    # Convertir la imagen a escala de grises
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Definir el diccionario de marcadores
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

    # Definir los parámetros del detector
    parameters = cv2.aruco.DetectorParameters()

    # Definimos los parámetros de la cámara
    camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])
    distortion = np.array([0, 0, 0, 0])

    # Detectamos los códigos ArUco en el cuadro
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Leemos la imagen
    frame = cv2.aruco.drawDetectedMarkers(image, corners, ids)

    # Si se encontraron códigos ArUco, calculamos las coordenadas en 3D
    if len(corners) > 0:
        # Calculamos las coordenadas de los códigos ArUco
        _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, distortion)

        # Dibujamos los ejes de coordenadas para cada código ArUco
        for i in range(len(ids)):
            print(f"Aruco {ids[i]} detectado.")
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Calcular el promedio de las coordenadas x e y de los vértices
            corners_first_tag = corners[0]
            avg_x = int(np.mean(corners_first_tag[:, :, 0]))
            avg_y = int(np.mean(corners_first_tag[:, :, 1]))

            aruco = Float32MultiArray()
            # Asinar valores i = id, avg_x = x, avg_y = y al mensaje a publicar
            aruco.data = [ids[i], avg_x, avg_y]
            #todo Verificar 3 veces la deteccion del codigo para que no publique cosas random
            pub.publish(aruco)
            #print("Se detecto ARUCO con ID: ",i)
        
    
    # Mostrar la imagen en pantalla completa       
    #image = cv2.resize(image, (0, 0), fx=2, fy=2)
    # cv2.namedWindow("ZED Camera", cv2.WND_PROP_FULLSCREEN)
    # cv2.setWindowProperty("ZED Camera", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    # cv2.imshow("ZED Camera", image)
    # cv2.waitKey(1)  # Espera un breve momento para procesar los eventos de ventana


    # except Exception as e:
    #     rospy.logerr(e)
def main():
    global pub
    # Inicializa el nodo suscriptor
    #rospy.init_node('aruco_detection_node', anonymous=True)
    rospy.init_node('aruco_detection_node')
    print("Aruco_detection_node has started")

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
