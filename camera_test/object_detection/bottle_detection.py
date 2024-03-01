#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import camera_functions as cam 
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import sys

# ----------- READ DNN MODEL -----------
# Obtener los nombres de archivo prototxt y model de los argumentos de línea de comandos
# print(rospy.get_param("prototxt"))
# prototxt = rospy.get_param('prototxt')
# model = rospy.get_param('model')
prototxt = "/home/samu/camera_tests/src/camera_test/src/object_detection/MobileNetSSD_deploy.prototxt.txt"
# Weights
model = "/home/samu/camera_tests/src/camera_test/src/object_detection/MobileNetSSD_deploy.caffemodel"
print(prototxt)
# Class labels
classes = {0:"background", 1:"aeroplane", 2:"bicycle",
          3:"bird", 4:"boat",
          5:"bottle", 6:"bus",
          7:"car", 8:"cat",
          9:"chair", 10:"cow",
          11:"diningtable", 12:"dog",
          13:"horse", 14:"motorbike",
          15:"person", 16:"pottedplant",
          17:"sheep", 18:"sofa",
          19:"train", 20:"tvmonitor"}
# Load the model
assert os.path.exists(prototxt)
assert os.path.exists(model)
net = cv2.dnn.readNetFromCaffe(prototxt, model)


def callback_imagen(msg):
    # Convierte el mensaje ROS de imagen a un objeto OpenCV
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    cam.object_detection(image)
    height, width, _ = image.shape
    image_resized = cv2.resize(image, (300, 300))
    # Create a blob
    blob = cv2.dnn.blobFromImage(image_resized, 0.007843, (300, 300), (127.5, 127.5, 127.5))

    # Perform object detection
    net.setInput(blob)
    detections = net.forward()

    # Process detections
    for detection in detections[0][0]:
        if detection[2] > 0.45 and int(detection[1]) == 5:  # Check if it's a bottle
            label = classes[int(detection[1])]
            box = detection[3:7] * [width, height, width, height]
            x_start, y_start, x_end, y_end = int(box[0]), int(box[1]), int(box[2]), int(box[3])
            
            # Draw bounding box and label on the image
            cv2.rectangle(image, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
            cv2.putText(image, "Conf: {:.2f}".format(detection[2] * 100), (x_start, y_start - 5), 1, 1.2, (255, 0, 0), 2)
            cv2.putText(image, label, (x_start, y_start - 25), 1, 1.2, (255, 0, 0), 2)

    # Display the resulting image
    cv2.imshow("Image", image)
    cv2.waitKey(1)
    #cv2.destroyAllWindows()



def main():
    # Inicializa el nodo suscriptor
    rospy.init_node('bottle_detection_node', anonymous=True)

    # Suscríbete al tópico /camera/image_raw con el tipo de mensaje Image
    rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color", Image, callback_imagen)
    rospy.wait_for_message("/zed2i/zed_node/rgb/image_rect_color", Image)  # Espera a que llegue el primer mensaje

    # Mantiene el nodo en ejecución
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
