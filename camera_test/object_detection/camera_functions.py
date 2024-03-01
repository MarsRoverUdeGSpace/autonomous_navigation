import cv2
import numpy as np
import os
def object_detection(image):
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


def red():
    # ----------- READ DNN MODEL -----------
    # Obtener los nombres de archivo prototxt y model de los argumentos de línea de comandos
    # print(rospy.get_param("prototxt"))
    # prototxt = rospy.get_param('prototxt')
    # model = rospy.get_param('model')
    prototxt = "/home/samu/camera_tests/src/camera_test/object_detection/MobileNetSSD_deploy.prototxt.txt"
    #print(prototxt)
    # Weights
    model = "/home/samu/camera_tests/src/camera_test/object_detection/MobileNetSSD_deploy.caffemodel"
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
    return net, classes

def bottle_detection(image):
    # Convierte el mensaje ROS de imagen a un objeto OpenCV
    height, width, _ = image.shape
    image_resized = cv2.resize(image, (300, 300))
    # Create a blob
    blob = cv2.dnn.blobFromImage(image_resized, 0.007843, (300, 300), (127.5, 127.5, 127.5))
    neu_net = red()
    net = neu_net[0]
    classes = neu_net[1]
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