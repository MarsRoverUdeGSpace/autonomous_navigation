#!/usr/bin/env python3

import rospy
import actionlib

from nav_msgs.msg import Odometry
from gps_navigation.msg import GPSGoalAction
from sensor_msgs.msg import NavSatFix
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geographic_msgs.msg import GeoPoint
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion

class GPSNavigationServer:
    def __init__(self):
        rospy.init_node('gps_navigation_server')
        self.action_server = actionlib.SimpleActionServer('gps_navigation', GPSGoalAction, self.execute, False)
        self.marker_pub = rospy.Publisher('marker_topic', Marker, queue_size=10)
        self.action_server.start()

        # Subscribe to the GPS fix topic
        # rospy.Subscriber('fix', NavSatFix, self.gps_fix_callback)  # Assuming the topic name is 'fix'
        self.gps_fix_callback()

        # Subscribe to the odometry topic
        rospy.Subscriber('odom', Odometry, self.odometry_callback)  #todo Change topic name
        self.robot_pose = None

        

        rospy.loginfo('GPS Navigation Server Initialized')

    def odometry_callback(self, msg):
        # Update the robot's pose based on odometry data
        self.robot_pose = msg.pose.pose

    def gps_fix_callback(self):
        # Wait for the topic to become available
        # rospy.wait_for_message("odom",Odometry, 10)

        # Read the latest message from the topic
        # latest_message = rospy.wait_for_message(Odometry, timeout=None)

        # Extract GPS coordinates from the fix message
        self.robot_pose = GeoPoint()
        # self.robot_pose.latitude = fix.latitude
        # self.robot_pose.longitude = fix.longitude
        # self.robot_pose.altitude = fix.altitude

        #! Probando
        self.robot_pose.longitude = -122.4194  # Example longitude
        self.robot_pose.latitude = 30.7749  # Example latitude
        self.robot_pose.altitude = 0  # Example altitude

        self.publish_robot_pose_marker("robot_pose", self.robot_pose.longitude, self.robot_pose.latitude)

        # Log the received GPS coordinates
        rospy.loginfo('Robot pose initialized: Lat={}, Lon={}, Alt={}'.format(self.robot_pose.latitude, self.robot_pose.longitude, self.robot_pose.altitude))

    def publish_robot_pose_marker(self, name, x, y):
        if self.robot_pose:
            marker = Marker()
            marker.header.frame_id = "map"  # Assuming the odometry frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = name
            marker.id = 0
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            marker.pose = Point(x,y,0)

            marker.scale.x = 1.0  # Length of the arrow
            marker.scale.y = 0.1  # Diameter of the arrow
            marker.scale.z = 0.1  # Height of the arrow
            marker.color.a = 1.0  # Alpha
            marker.color.r = 0.0  # Red
            marker.color.g = 1.0  # Green
            marker.color.b = 0.0  # Blue
            self.marker_pub.publish(marker)

    def execute(self, goal):
        rospy.loginfo('Received GPS Goal: Lat={}, Lon={}'.format(goal.latitud, goal.longitud))

        # Ensure robot's pose is available
        if self.robot_pose is None:
            rospy.logwarn('Robot pose is not available. Skipping goal execution.')
            self.action_server.set_aborted()
            return

        # Calculate relative position of GPS goal w.r.t. robot's pose
        relative_x = goal.x - self.robot_pose.position.x
        relative_y = goal.y - self.robot_pose.position.y

        # If robot's orientation (heading) is relevant, account for it
        # For example, you may need to rotate the relative position based on robot's orientation
        # Convert relative position to robot's frame of reference
        # Here, we assume no rotation for simplicity
        robot_x = relative_x
        robot_y = relative_y

        self.publish_robot_pose_marker("goal_pose", robot_x, robot_y)

        # Now you have the GPS goal coordinates in the robot's frame of reference
        rospy.loginfo('Goal in robot frame: x={}, y={}'.format(robot_x, robot_y))

        # Create a MoveBaseGoal with the transformed coordinates
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = 'map'  # Assuming the goal is in the map frame
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose.position.x = robot_x
        move_base_goal.target_pose.pose.position.y = robot_y
        move_base_goal.target_pose.pose.orientation.w = 1  # Assuming no rotation

        # Send the goal to move_base
        # (You may need to use the action client to communicate with move_base)
        # (Refer to move_base documentation for how to send goals)
        
        self.action_server.set_succeeded()  # Indicate that the action has been completed


if __name__ == '__main__':
    try:
        GPSNavigationServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
