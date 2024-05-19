#!/usr/bin/env python3
import sys
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
import sensor_msgs.point_cloud2 as pc2
import rospy
import tf.transformations
import numpy as np

def msg_tf(transformStampedMsg, headerFrame, childFrame,
              translationX=0, translationY=0, translationZ=0,
              quaternion=(0, 0, 0, 1)):
    transformStampedMsg.header.stamp = rospy.Time.now()
    transformStampedMsg.header.frame_id = headerFrame
    transformStampedMsg.child_frame_id = childFrame

    transformStampedMsg.transform.translation.x = translationX
    transformStampedMsg.transform.translation.y = translationY
    transformStampedMsg.transform.translation.z = translationZ

    transformStampedMsg.transform.rotation.x = quaternion[0]
    transformStampedMsg.transform.rotation.y = quaternion[1]
    transformStampedMsg.transform.rotation.z = quaternion[2]
    transformStampedMsg.transform.rotation.w = quaternion[3]
    return transformStampedMsg

def aruco_tf(position,t, id):

    t.header.frame_id = "zed2i_base_link"
    t.child_frame_id = "AR_" + str(id)
    t.transform.translation.x = position[0]
    t.transform.translation.y = position[1]
    t.transform.translation.z = position[2]
    q = quaternion_from_euler(0, 0, 0)  # Convertir ángulos de Euler a cuaternión
    t.transform.rotation.x = q[0]
    t.transform.rotation.y =q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t

def callback_point_cloud(msg):
    global point_cloud_data
    global avg_x
    global avg_y
    # actual = rospy.get_time()
    if avg_x != 0 and avg_y != 0:
        point_cloud_data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False))
    else:
        print("No se pudo obtener la nube de puntos")
    # print("Tiempo:",rospy.get_time() - actual)

def callback_aruco(msg):
    global avg_x, avg_y
    global punto_aux, punto
    global point_cloud_data
    global tf_broadcaster
    t = TransformStamped()
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
            if not np.isnan(point_cloud_data[index][0]):  # Verificar si la coordenada x no es un número
                punto = point_cloud_data[index]
                punto_aux = punto
                id = "AR"+str(id)
                print("Aruco in (x, y, z):", punto, "Aruco_ID: ",id)
                t = msg_tf(t,"zed2i_base_link",id,
                            punto[0],punto[1],punto[2],
                            quaternion_from_euler(0,0,0))
                t.header.stamp = rospy.Time.now()
                tf_broadcaster.sendTransformMessage(t)
            else:
                if punto_aux != 0:
                    id = "AR"+str(id)
                    print("Aruco in (x, y, z):", punto, "Aruco_ID: ",id)
                    t = msg_tf(t,"zed2i_base_link",id,
                                punto_aux[0],punto_aux[1],punto_aux[2],
                                quaternion_from_euler(0,0,0))
                    t.header.stamp = rospy.Time.now()
                    tf_broadcaster.sendTransformMessage(t)
                else:
                    print("The point has not been detected correctly in the point cloud.")
        else:
            print("Coordinates are outside the range of the point cloud.")
    



def main():
    global punto_aux, punto
    global avg_x, avg_y
    global tf_broadcaster
    global point_cloud_data
    point_cloud_data = 0
    punto_aux, punto = 0, 0
    avg_y, avg_x = 0, 0
    # Inicializa el nodo suscriptor
    rospy.init_node("aruco_position_node")
    print("aruco_position_node has started")
    # rate = rospy.Rate(10)

    tf_broadcaster = tf.TransformBroadcaster()

    rospy.wait_for_message("/zed2i/zed_node/point_cloud/cloud_registered", PointCloud2)  # Espera a que llegue el primer mensaje
    rospy.wait_for_message("/Aruco", Float32MultiArray)  # Espera a que llegue el primer mensaje
    # Subscribers
    rospy.Subscriber("/Aruco", Float32MultiArray, callback_aruco)
    rospy.Subscriber("/zed2i/zed_node/point_cloud/cloud_registered", PointCloud2, callback_point_cloud)
    
    while not rospy.is_shutdown():
        pass

    # Keep node activated
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
