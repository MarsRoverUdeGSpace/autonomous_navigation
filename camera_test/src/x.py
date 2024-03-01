#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def image_callback(msg):
    step = msg.step
    ancho = msg.width
    encoding = msg.encoding
    #print(ancho)
    #print(msg.height)
    #print(encoding)
    #print(step)
    #print(msg.data)
    #print(len(msg.data)/4/360)
    imagen =np.array(msg.data)
    #print(np.shape(imagen))
    mi_lista = [1, 2, [3, 4], 5]
    for elemento in mi_lista:
        print(len(mi_lista))
        if isinstance(elemento, list):
            print("La lista es una lista anidada.")
            break
    else:
        print("La lista no es una lista anidada.")

def main():
    rospy.init_node('image_subscriber', anonymous=True)
    # Suscribirse al tema que publica la imagen
    rospy.Subscriber("/zed2i/zed_node/left/image_rect_color", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
