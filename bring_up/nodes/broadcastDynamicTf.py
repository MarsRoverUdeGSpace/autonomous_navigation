#!/usr/bin/env python3

import rospy
import tf
import functools
from nav_msgs.msg import Odometry
import geometry_msgs
import tf.transformations

# Get odometry data
def callbackOdometry(msg, headerFrame, childFrame):
    global transformStampedMsg
    translationX = msg.pose.pose.position.x
    translationY = msg.pose.pose.position.y
    translationZ = msg.pose.pose.position.z
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    transformStampedMsg = msg_tf(transformStampedMsg, headerFrame, childFrame,
                                 translationX, translationY, translationZ,
                                 quaternion)

# Fill transformStampedMsg
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

def main():
    global transformStampedMsg
    # Initialize node
    rospy.init_node('baseLinkTfBroadcaster')
    # Define rate
    rate = rospy.Rate(3)

    # Get params from the launch file
    topic = rospy.get_param('~topic', "/odom")
    header = rospy.get_param('~headerId', "zed2i_base_link")
    child = rospy.get_param('~childID', "arsi")
    translationX = rospy.get_param('~translationX',0.0)
    translationY = rospy.get_param('~translationY',0.0)
    translationZ = rospy.get_param('~translationZ',0.)
    eulerR = rospy.get_param('~eulerR',0.0)
    eulerP = rospy.get_param('~eulerP',0.0)
    eulerY = rospy.get_param('~eulerY',0.0)

    # Transform euler cordinates to quaternion
    quaternion = tf.transformations.quaternion_from_euler(eulerR, eulerP, eulerY)
    # Create a callback_partial to use parameters in callback function
    callback_partial = functools.partial(callbackOdometry, headerFrame=header, childFrame=child)
    #Subscribe to odometry data
    rospy.Subscriber(topic, Odometry, callback_partial)

    # Create an object type TransformBroadcaster to public dynamic tf
    dynamicBroadcaster = tf.TransformBroadcaster()
    transformStampedMsg = geometry_msgs.msg.TransformStamped()
    transformStampedMsg = msg_tf(transformStampedMsg, header, child, 
                                 translationX, translationY, translationZ,
                                 quaternion)
    
    while not rospy.is_shutdown():
        # Update stamp time for every loop
        transformStampedMsg.header.stamp = rospy.Time.now()
        # Publishing dynamic tf
        dynamicBroadcaster.sendTransformMessage(transformStampedMsg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)
