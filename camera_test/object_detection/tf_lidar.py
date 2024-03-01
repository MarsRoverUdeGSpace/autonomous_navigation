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

def lidar_tf():
    global t

    t.header.frame_id = "ZED_link"
    t.child_frame_id = "Laser"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    q = quaternion_from_euler(0, 0, 1)  # Convertir ángulos de Euler a cuaternión
    t.transform.rotation.x = q[0]
    t.transform.rotation.y =q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    #transform.transform.translation.w = 1


def main():
    global t

    t = TransformStamped()
    # Inicializa el nodo suscriptor
    rospy.init_node("aruco_position_node")
    print("El nodo aruco_position ha sido iniciado")
    rate = rospy.Rate(10)

    tf_broadcaster = tf.TransformBroadcaster()

    #Mientras que no se detenga el nodo
    while not rospy.is_shutdown():

        t.header.stamp = rospy.Time.now()
        tf_broadcaster.sendTransform(t)
        rate.sleep()
    else:
        print("La nube de puntos no se ha recibido correctamente.")

    # Mantiene el nodo en ejecución
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
