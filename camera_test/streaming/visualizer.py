#!/usr/bin/env python3
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy


def callback_imagen(msg):
    # Convierte el mensaje ROS de imagen a un objeto OpenCV
    bridge = CvBridge()
    imagen = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # Procesa la imagen recibida (por ejemplo, muestra la imagen)
    cv2.imshow("ZED Camera", imagen)
    cv2.waitKey(1)  # Espera un breve momento para procesar los eventos de ventana

def main():
    # Inicializa el nodo suscriptor
    rospy.init_node('nodo_suscriptor', anonymous=True)

    # Suscríbete al tópico /camera/image_raw con el tipo de mensaje Image
    rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, callback_imagen)    
    rospy.wait_for_message("/zed2i/zed_node/rgb/image_rect_color",Image)


    while not rospy.is_shutdown:
        pass
    # Mantiene el nodo en ejecución
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
