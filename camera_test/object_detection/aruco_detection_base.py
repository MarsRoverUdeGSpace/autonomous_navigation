#!/usr/bin/env python3
import cv2
from sensor_msgs.msg import Image
import camera_functions as cam 
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
import rospy
import numpy as np
def callback_imagen(msg):
    global pub
    # a = msg.data
    # print(len(a))
    # Convierte el mensaje ROS de imagen a un objeto OpenCV
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    cam.bottle_detection(image)
    #cam.object_detection(image)

    #alto, ancho, canales = image.shape
    #print("Dimensiones de la imagen: {}x{}x{}".format(ancho, alto, canales))
    # Convertimos el cuadro a escala de grises
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
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Calcular el promedio de las coordenadas x e y de los vértices
            corners_first_tag = corners[0]
            avg_x = int(np.mean(corners_first_tag[:, :, 0]))
            avg_y = int(np.mean(corners_first_tag[:, :, 1]))

            aruco = Float32MultiArray()
            # Asinar valores i = id, avg_x = x, avg_y = y al mensaje a publicar
            aruco.data = [ids[i], avg_x, avg_y]
            pub.publish(aruco)
            #print("Se detecto ARUCO con ID: ",i)
        

    # Mostrar la imagen en pantalla completa
    cv2.namedWindow("ZED Camera", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("ZED Camera", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("ZED Camera", image)
    cv2.waitKey(1)  # Espera un breve momento para procesar los eventos de ventana

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
