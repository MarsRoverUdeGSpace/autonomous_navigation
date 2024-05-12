#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import geometry_msgs

# Fill transformStampedMsg
def msg_tf(transformStampedMsg, headerFrame, childFrame,
              translationX=0,translationY=0,translationZ=0,eulerR=0,eulerP=0,eulerY=0):
    
    transformStampedMsg.header.stamp = rospy.Time.now()
    transformStampedMsg.header.frame_id = headerFrame
    transformStampedMsg.child_frame_id = childFrame

    transformStampedMsg.transform.translation.x = translationX
    transformStampedMsg.transform.translation.y = translationY
    transformStampedMsg.transform.translation.z = translationZ

    quaternion = tf.transformations.quaternion_from_euler(eulerR, eulerP, eulerY)
    transformStampedMsg.transform.rotation.x = quaternion[0]
    transformStampedMsg.transform.rotation.y = quaternion[1]
    transformStampedMsg.transform.rotation.z = quaternion[2]
    transformStampedMsg.transform.rotation.w = quaternion[3]
    return transformStampedMsg

def main():
    # Initialize node
    rospy.init_node('mapTfBroadcasterNode')

    # Get params from the launch file
    header = rospy.get_param('~headerId', "map")
    child = rospy.get_param('~childID', "odom")
    translationX = rospy.get_param('~translationX',0.0)
    translationY = rospy.get_param('~translationY',0.0)
    translationZ = rospy.get_param('~translationZ',0.27)
    eulerR = rospy.get_param('~eulerR',0.0)
    eulerP = rospy.get_param('~eulerP',0.0)
    eulerY = rospy.get_param('~eulerY',0.0)

    #Create an object type StaticTransformBroadcaster 
    staticBroadcaster = tf2_ros.StaticTransformBroadcaster()
    transformStampedMsg = geometry_msgs.msg.TransformStamped()
    transformStampedMsg = msg_tf(transformStampedMsg, header, child, 
                                 translationX, translationY, translationZ,
                                 eulerR, eulerP, eulerY)
    # Publishig static Tf
    staticBroadcaster.sendTransform(transformStampedMsg)
    
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print(rospy.ROSInterruptException)