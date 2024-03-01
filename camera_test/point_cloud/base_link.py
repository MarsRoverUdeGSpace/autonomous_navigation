#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

def create_reference_frame():
    # Inicializar el nodo de ROS
    rospy.init_node('reference_frame_publisher', anonymous=True)

    # Crear un objeto TransformBroadcaster
    br = tf2_ros.TransformBroadcaster()

    # Crear un objeto TransformStamped
    t = geometry_msgs.msg.TransformStamped()

    # Establecer el marco de referencia base
    t.header.frame_id = 'base_link'  # Cambia 'map' al marco de referencia que desees como base

    # Establecer el marco de referencia objetivo
    t.child_frame_id = 'rover'  # Cambia 'base_link' al marco de referencia que deseas crear

    # Configurar la posición y orientación (en este ejemplo, la transformación es la identidad)
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 1.0
    roll, pitch, yaw = 0.0, 0.0, 1.5708
    orientaton_quaternion = quaternion_from_euler(roll, pitch, yaw)
    t.transform.rotation.x = orientaton_quaternion[0]
    t.transform.rotation.y = orientaton_quaternion[1]
    t.transform.rotation.z = orientaton_quaternion[2]
    t.transform.rotation.w = orientaton_quaternion[3]

    # Publicar la transformación continuamente
    rate = rospy.Rate(1)  # Publicar a una velocidad de 1 Hz
    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        br.sendTransform(t)
        rate.sleep()

if __name__ == '__main__':
    try:
        create_reference_frame()
    except rospy.ROSInterruptException:
        pass
