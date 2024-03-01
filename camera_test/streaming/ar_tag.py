#!/usr/bin/env python3
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv2 import aruco
import rospy


def callback_imagen(msg):
    # Convierte el mensaje ROS de imagen a un objeto OpenCV
    bridge = CvBridge()
    imagen = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

    # Initialize ArUco detector parameters
    parameters = cv2.aruco.DetectorParameters_create()

    # Load the input image
    image = cv2.imread(imagen)

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers in the image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Draw detected markers on the image
    image_markers = cv2.aruco.drawDetectedMarkers(image.copy(), corners, ids)

    # Display the image with detected markers
    cv2.imshow('ArUco Tag Detection', image_markers)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main():
    # Inicializa el nodo suscriptor
    rospy.init_node('nodo_suscriptor', anonymous=True)

    # Suscríbete al tópico /camera/image_raw con el tipo de mensaje Image
    rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, callback_imagen)    

    # Mantiene el nodo en ejecución
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
