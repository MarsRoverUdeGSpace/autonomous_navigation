#!/usr/bin/env python3

import rospy
import actionlib
from gps_navigation.msg import GPSGoalAction, GPSGoalGoal
from geometry_msgs.msg import Point

def send_goal():
    # Initialize ROS node
    rospy.init_node('gps_navigation_client')

    # Create an action client
    client = actionlib.SimpleActionClient('gps_navigation', GPSGoalAction)
    rospy.loginfo('Waiting for action server...')
    client.wait_for_server()

    # Create a goal message
    goal = GPSGoalGoal()
    goal.longitud = -122.4194  # Example longitude
    goal.latitud = 37.7749  # Example latitude
    goal.altitud = 0  # Example altitude

    # Send the goal to the action server
    rospy.loginfo('Sending goal...')
    client.send_goal(goal)

    # Wait for the result
    rospy.loginfo('Waiting for result...')
    client.wait_for_result()

    # Print the result
    result = client.get_result()
    rospy.loginfo('Result: {}'.format(result))

if __name__ == '__main__':
    try:
        send_goal()
    except rospy.ROSInterruptException:
        pass
