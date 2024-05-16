#!/usr/bin/env python3
import time
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import sensor_msgs.point_cloud2 as pc2
import rospy
import tf2_ros as tf
import numpy as np
from tf2_msgs.msg import TFMessage

# def tf_callback(msg):
    
#     for transform in msg.transforms:
#         if transform.child_frame_id == "AR_0" and transform.header.frame_id == "ZED_link":
#             print("Transformación encontrada:")
#             print(transform)
#         if transform.header.frame_id == "chassis_link" and transform.child_frame_id == "ZED_link":
#             print(transform)

def aruco_tf(position):
    global id, t

    t = TransformStamped()
    t.header.frame_id = "zed2i_base_link"
    t.child_frame_id = "AR_" + str(id)
    t.transform.translation.x = position[0]
    t.transform.translation.y = position[1]
    t.transform.translation.z = position[2]
    q = quaternion_from_euler(0, 0, 1)  # Convertir ángulos de Euler a cuaternión
    t.transform.rotation.x = q[0]
    t.transform.rotation.y =q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    #transform.transform.translation.w = 1

def callback_point_cloud(msg):
    global point_cloud_data
    global avg_x
    global avg_y
    if avg_x != 0 and avg_y != 0:
        point_cloud_data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False))
    else:
        print("No se pudo obtener la nube de puntos")

def callback_aruco(msg):
    global avg_x
    global avg_y
    global id
    global tf_broadcaster
    global point_cloud_data
    punto_aux = 0
    avg_y, avg_x = 0, 0
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
                print("Aruco in (x, y, z):", punto, "Aruco_ID: ",id)
                aruco_tf(punto)
                t.header.stamp = rospy.Time.now()
                tf_broadcaster.sendTransform(t)
                #print(index)
                #print(point_cloud_data[index])
            
                # print("ID: ",id)
                # print("avg_x: ",avg_x)
                # print("avg_y: ",avg_y)
            else:
                if punto_aux != 0:
                    print("Aruco in (x, y, z):", punto, "Aruco_ID: ",id)
                    aruco_tf(punto_aux)
                    t.header.stamp = rospy.Time.now()
                    tf_broadcaster.sendTransform(t)
                else:
                    print("The point has not been detected correctly in the point cloud.")
        else:
            print("Coordinates are outside the range of the point cloud.")



def main():
    global avg_x
    global avg_y
    global t
    global id
    global tf_broadcaster
    global point_cloud_data
    point_cloud_data = 0
    # Inicializa el nodo suscriptor
    rospy.init_node("aruco_position_node")
    print("El nodo aruco_position ha sido iniciado")
    rate = rospy.Rate(10)

    tf_broadcaster = tf.TransformBroadcaster()

    # Subscriptor al tópico /zed2i/zed_node/point_cloud/cloud_registered con el tipo de mensaje Point_cloud2
    aruco_sub = rospy.Subscriber("/Aruco", Float32MultiArray, callback_aruco)
    
    rospy.wait_for_message("/Aruco", Float32MultiArray)  # Espera a que llegue el primer mensaje


    # rospy.Subscriber("/tf", TFMessage, tf_callback)
    # rospy.wait_for_message("/tf", TFMessage)  # Espera a que llegue el primer mensaje

    # Subscriptor al tópico /zed2i/zed_node/point_cloud/cloud_registered con el tipo de mensaje Point_cloud2
    rospy.Subscriber("/zed2i/zed_node/point_cloud/cloud_registered", PointCloud2, callback_point_cloud)
    
    rospy.wait_for_message("/zed2i/zed_node/point_cloud/cloud_registered", PointCloud2)  # Espera a que llegue el primer mensaje
    
    #rospy.loginfo("Unsubscribed from topic '/zed2i/zed_node/point_cloud/cloud_registered'")
    
    #rospy.Timer(rospy.Duration(1), callback_point_cloud)
    #Mientras que no se detenga el nodo
    while not rospy.is_shutdown():
        # last_time = rospy.Time.now().to_sec()
        # try:
        #     rospy.wait_for_message("/Aruco", Float32MultiArray,timeout=5)  # Espera a que llegue el primer mensaje
        # except rospy.exceptions.ROSException as e:
        #     print("An ARUCO tag has not been detected")

        #     #tf_broadcaster.unregister()
        # rate.sleep()  # Espera para cumplir la frecuencia deseada
        pass
    else:
        print("La nube de puntos no se ha recibido correctamente.")

    # Mantiene el nodo en ejecución
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
