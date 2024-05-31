#!/usr/bin/env python3
import rospy
from std_msgs.msg import ColorRGBA
import sys

def send_color_command(color_param):
    pub = rospy.Publisher('led_color', ColorRGBA, queue_size=10)
    rospy.init_node('led_color_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    color = ColorRGBA()
    
    # Set the color based on the parameter
    if color_param == "red":
        color.r = 1.0
        color.g = 0.0
        color.b = 0.0
    elif color_param == "green":
        color.r = 0.0
        color.g = 1.0
        color.b = 0.0
    elif color_param == "blue":
        color.r = 0.0
        color.g = 0.0
        color.b = 1.0
    else:
        rospy.logerr("Invalid color parameter. Use 'red', 'green', or 'blue'.")
        return

    color.a = 1.0

    while not rospy.is_shutdown():
        pub.publish(color)
        rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        rospy.logerr("Usage: change_led_color.py [color]")
    else:
        try:
            send_color_command(sys.argv[1])
        except rospy.ROSInterruptException:
            pass
