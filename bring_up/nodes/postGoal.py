#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import pyproj

class GNSSConverterNode:
    def __init__(self):
        rospy.init_node('gnss_converter_node', anonymous=True)
        
        # Define the GNSS to Cartesian converter
        self.gnss_proj = pyproj.Proj(proj='latlong', datum='WGS84')
        self.cartesian_proj = pyproj.Proj(proj='utm', zone=33, datum='WGS84')  # Adjust zone as per your location
        
        # Define the goal publisher
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        rospy.loginfo("GNSS Converter Node Initialized")

    def convert_gnss_to_cartesian(self, latitude, longitude):
        # Convert GNSS coordinates to Cartesian coordinates
        x, y = pyproj.transform(self.gnss_proj, self.cartesian_proj, longitude, latitude)
        return x, y

    def send_goal(self, x, y):
        # Create and publish the goal message
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"  # or any other appropriate frame_id
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.goal_publisher.publish(goal_msg)

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            # Example GNSS coordinates (you should replace these with your actual coordinates)
            gnss_latitude = 48.8566  # Example latitude (e.g., Paris)
            gnss_longitude = 2.3522  # Example longitude (e.g., Paris)
            
            # Convert GNSS coordinates to Cartesian coordinates
            cartesian_x, cartesian_y = self.convert_gnss_to_cartesian(gnss_latitude, gnss_longitude)
            
            # Send the goal to Move Base
            self.send_goal(cartesian_x, cartesian_y)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        gnss_converter_node = GNSSConverterNode()
        gnss_converter_node.run()
    except rospy.ROSInterruptException:
        pass
