#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
from tf.transformations import quaternion_from_euler

def callback_aruco_tf(msg):
    global tf_broadcaster
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "zed2i_base_link"
    t.child_frame_id = "AR_" + str(int(msg.data[0]))
    t.transform.translation.x = msg.data[1]
    t.transform.translation.y = msg.data[2]
    t.transform.translation.z = msg.data[3]
    q = quaternion_from_euler(0, 0, 1)  # Convertir ángulos de Euler a cuaternión
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    tf_broadcaster.sendTransform(t)
    print(t)

def main():
    global tf_broadcaster
    rospy.init_node("aruco_tf_node")
    print("aruco_tf_node has started")

    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rospy.Subscriber("/Aruco_tf_info", Float32MultiArray, callback_aruco_tf)

    while not rospy.is_shutdown():
        pass
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
