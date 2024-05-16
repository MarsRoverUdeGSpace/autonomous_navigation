#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
import sensor_msgs.point_cloud2 as pc2
import rospy
import numpy as np
from tf2_msgs.msg import TFMessage

def callback_point_cloud(msg):
    global point_cloud_data
    global avg_x
    global avg_y
    global id
    if id != -1:
        if avg_x != 0 and avg_y != 0:
            point_cloud_data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False))
        else:
            print("No se pudo obtener la nube de puntos")

def callback_aruco(msg):
    global avg_x
    global avg_y
    global id
    global pub_tf_info
    global point_cloud_data
    global punto_aux
    global punto
    point_cloud_width = 427
    point_cloud_height = 240
    id = int(msg.data[0])
    avg_x = int(msg.data[1])
    avg_y = int(msg.data[2])
    if avg_y != 0 and avg_x != 0 and point_cloud_data != 0:
                # Verifica que las coordenadas estén dentro del rango de la nube de puntos
                if 0 <= avg_x < point_cloud_width and 0 <= avg_y < point_cloud_height:
                    # Obtén las coordenadas (x, y, z) del punto en la nube de puntos
                    index = avg_y * point_cloud_width + avg_x
                    # print("INDEX: ",index)
                    if not np.isnan(point_cloud_data[index][0]):  # Verificar si la coordenada x no es un número
                        punto = point_cloud_data[index]
                        punto_aux = punto
                        print("Aruco in (z, x, y):", punto, "Aruco_ID: ",id)
                        aruco = Float32MultiArray()
                        id = float(id)
                        aruco.data = [id,punto[0],punto[1],punto[2]]
                        # print(aruco)
                        pub_tf_info.publish(aruco)
                    else:
                        if punto_aux != 0 and type(id) == float:
                            print("Aruco in (z, x, y):", punto, "Aruco_ID: ",id)
                            aruco = Float32MultiArray()
                            aruco.data = [id,punto_aux]
                            # print(aruco)
                            pub_tf_info.publish(aruco)
                        else:
                            print("The point has not been detected correctly in the point cloud.")
                else:
                    print("Coordinates are outside the range of the point cloud.")

def main():
    global avg_x
    global avg_y
    global point_cloud_data
    global t
    global id
    global pub_tf_info
    global punto_aux
    global punto
    punto_aux, punto = 0 , 0 
    point_cloud_data = 0
    avg_y, avg_x = 0, 0
    # Inicializa el nodo suscriptor
    rospy.init_node("aruco_position_node")
    print("aruco_position_node has started")
    rate = rospy.Rate(6)

    rospy.wait_for_message("/zed2i/zed_node/point_cloud/cloud_registered", PointCloud2)  # Espera a que llegue el primer mensaje

    print("Waiting for aruco detection")
    rospy.wait_for_message("/Aruco", Float32MultiArray)  # Espera a que llegue el primer mensaje

    # Subscriptor al tópico /zed2i/zed_node/point_cloud/cloud_registered con el tipo de mensaje Point_cloud2
    aruco_sub = rospy.Subscriber("/Aruco", Float32MultiArray, callback_aruco)
    

    # Subscriptor al tópico /zed2i/zed_node/point_cloud/cloud_registered con el tipo de mensaje Point_cloud2
    rospy.Subscriber("/zed2i/zed_node/point_cloud/cloud_registered", PointCloud2, callback_point_cloud)
    pub_tf_info = rospy.Publisher("/Aruco_tf_info",Float32MultiArray,queue_size=10)

    #Mientras que no se detenga el nodo
    while not rospy.is_shutdown():
        pass

    # Mantiene el nodo en ejecución
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
