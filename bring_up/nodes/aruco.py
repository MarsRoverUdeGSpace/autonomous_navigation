#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
 
bridge = CvBridge()

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        print(e)
    else:
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

rospy.init_node('image_listener')
image_topic = "/camera/color/image_raw"
rospy.Subscriber(image_topic, Image, image_callback)
rospy.spin()
