#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf

def odom_publisher():
    rospy.init_node('odom_publisher', anonymous=True)
    pub = rospy.Publisher('odom', Odometry, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    # Set initial pose
    pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))

    while not rospy.is_shutdown():
        # Update timestamp
        current_time = rospy.Time.now()

        # Publish odom message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose = pose
        pub.publish(odom_msg)

        # Broadcast TF transform
        odom_broadcaster = tf.TransformBroadcaster()
        odom_broadcaster.sendTransform(
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            current_time,
            "base_link",
            "odom"
        )

        # Sleep to maintain the desired rate
        rate.sleep()

if __name__ == '__main__':
    try:
        odom_publisher()
    except rospy.ROSInterruptException:
        pass
