#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf import TransformListener
from geographic_msgs.msg import GeoPoseStamped

class InitialPoseSetter:
    def __init__(self):
        rospy.init_node('initial_pose_setter')

        # Subscribe to GPS data
        self.gps_sub = rospy.Subscriber('<your_gps_topic>', NavSatFix, self.gps_callback)  #todo Cambiar por mensaje del GPS

        # Publish initial pose estimate to AMCL
        self.amcl_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

        # Create a transform listener
        self.tf_listener = TransformListener()

    def gps_callback(self, msg):
        try:
            # Transform GPS coordinates to map frame
            gps_pose = GeoPoseStamped()
            gps_pose.header.stamp = rospy.Time.now()
            gps_pose.pose.position.latitude = msg.latitude
            gps_pose.pose.position.longitude = msg.longitude
            gps_pose.pose.position.altitude = msg.altitude

            map_pose = self.tf_listener.transformPose("map", gps_pose)

            # Publish the transformed pose as initial pose to AMCL
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.stamp = rospy.Time.now()
            initial_pose.header.frame_id = "map"
            initial_pose.pose.pose.position = map_pose.pose.position
            initial_pose.pose.pose.orientation.w = 1.0  # Assuming no orientation from GPS
            initial_pose.pose.covariance[0] = 0.5  # Adjust covariance values if necessary
            initial_pose.pose.covariance[7] = 0.5  # Adjust covariance values if necessary
            initial_pose.pose.covariance[35] = 0.1  # Adjust covariance values if necessary

            self.amcl_pose_pub.publish(initial_pose)
            rospy.loginfo("Initial pose set to: {}".format(initial_pose.pose.pose))

            # Shutdown the node after setting the initial pose
            self.gps_sub.unregister()
            rospy.signal_shutdown("Initial pose set")
        except Exception as e:
            rospy.logwarn("Failed to process GPS data: {}".format(e))

if __name__ == '__main__':
    try:
        initial_pose_setter = InitialPoseSetter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
