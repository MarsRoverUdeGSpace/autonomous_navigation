#!/usr/bin/env python3

import rospy
import actionlib
import sys
import io
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import NavSatFix
import utm
import subprocess

infoApi = {
    "goal_node" : "send_goal_node is shutdown",
    "status_GPS" : "Stopped",
    "goal_GPS" : None,
    "initial_utm" : None,
    "goal_utm" : None,
    "error_utm" : None,
    "initial_gps_coordinates" : None,
    "initial_marker" : None,
    "goal_marker" : None
}

class MoveBaseGoalSender:
    def __init__(self, goal_lat, goal_lon):

        rospy.init_node('send_goal_node')
        rospy.loginfo("send_goal_node started.")

        ros_command = ['rosrun', 'bring_up', 'change_led_color.py', "blue"]
        subprcs = subprocess.Popen(ros_command)
        rospy.sleep(1)
        subprcs.terminate()

        # Initialize variables to store the initial GPS coordinates
        self.initial_lat = None
        self.initial_lon = None

        # Subscribe to the GPS topic to get the initial coordinates
        rospy.Subscriber('/fix', NavSatFix, self.gps_callback)

        # Publishers for initial and goal markers
        self.initial_marker_pub = rospy.Publisher('initial_pose_marker', PoseStamped, queue_size=10)
        self.goal_marker_pub = rospy.Publisher('goal_pose_marker', PoseStamped, queue_size=10)

        # Action client for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Set goal coordinates from arguments
        self.goal_lat = goal_lat
        self.goal_lon = goal_lon

        # Wait until we get the initial GPS coordinates
        rospy.loginfo("Waiting for initial GPS coordinates...")

        while self.initial_lat is None or self.initial_lon is None:
            rospy.sleep(0.1)

        rospy.loginfo("Received goal GPS coordinates: lat={}, lon={}".format(self.goal_lat, self.goal_lon))

        # Convert initial and goal GPS coordinates to UTM coordinates
        initial_utm = self.gps_to_utm(self.initial_lat, self.initial_lon)
        goal_utm = self.gps_to_utm(self.goal_lat, self.goal_lon)

        rospy.loginfo("Initial UTM: x={}, y={}, zone={}{}".format(initial_utm[0], initial_utm[1], initial_utm[2], initial_utm[3]))
        rospy.loginfo("Goal UTM: x={}, y={}, zone={}{}".format(goal_utm[0], goal_utm[1], goal_utm[2], goal_utm[3]))

        # Ensure the UTM coordinates are in the same zone
        if initial_utm[2] != goal_utm[2] or initial_utm[3] != goal_utm[3]:
            rospy.logerr("Initial and goal coordinates are in different UTM zones")
        else:
            # Calculate relative coordinates
            rel_x = goal_utm[0] - initial_utm[0]
            rel_y = goal_utm[1] - initial_utm[1]

            # Send the goal to move_base and publish the markers for RViz
            self.send_goal_and_publish_markers(0, 0, rel_x, rel_y)

    def gps_callback(self, msg):
        """Callback function to handle GPS data."""
        if self.initial_lat is None and self.initial_lon is None:
            self.initial_lat = msg.latitude
            self.initial_lon = msg.longitude
            rospy.loginfo("Received initial GPS coordinates: lat={}, lon={}".format(self.initial_lat, self.initial_lon))

    def gps_to_utm(self, lat, lon):
        """Convert GPS coordinates to UTM coordinates using the utm package."""
        u = utm.from_latlon(lat, lon)
        return u[0], u[1], u[2], u[3]  # Returns easting, northing, zone number, and zone letter

    def send_goal_and_publish_markers(self, initial_x, initial_y, goal_x, goal_y, z=0, w=1):
        """Send a goal to the move_base action server and publish markers for RViz."""
        # Create a SimpleActionClient for move_base
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        rospy.loginfo("Connected to move_base action server.")
        
        # Publish the initial and goal poses as PoseStamped messages for RViz
        initial_marker = PoseStamped()
        initial_marker.header.frame_id = "map"
        initial_marker.header.stamp = rospy.Time.now()
        initial_marker.pose = Pose(Point(initial_x, initial_y, 0), Quaternion(0, 0, z, w))

        goal_marker = PoseStamped()
        goal_marker.header.frame_id = "map"
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.pose = Pose(Point(goal_x, goal_y, 0), Quaternion(0, 0, z, w))

        # Wait for the publishers to connect
        while self.initial_marker_pub.get_num_connections() < 1 or self.goal_marker_pub.get_num_connections() < 1:
            rospy.sleep(0.1)

        self.initial_marker_pub.publish(initial_marker)

        rospy.loginfo("Published initial marker at: x={}, y={}".format(initial_x, initial_y))

        self.goal_marker_pub.publish(goal_marker)

        rospy.loginfo("Published goal marker at: x={}, y={}".format(goal_x, goal_y))

        # Create a MoveBaseGoal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(goal_x, goal_y, 0), Quaternion(0, 0, z, w))

        # Send the goal to move_base
        rospy.loginfo("Sending goal: x={}, y={}".format(goal_x, goal_y))

        client.send_goal(goal)
        #! Probando
        ros_command = ['rosrun', 'bring_up', 'change_led_color.py', "red"]
        subprcs = subprocess.Popen(ros_command)
        client.wait_for_result()
        subprcs.terminate()

        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully")
            ros_command = ['rosrun', 'bring_up', 'change_led_color.py', "green"]
            subprcs = subprocess.Popen(ros_command)
            rospy.sleep(0.5)
            subprcs.terminate()
            rospy.sleep(3)
            
        else:
            rospy.logwarn("Failed to reach the goal")
        
        ros_command = ['rosrun', 'bring_up', 'change_led_color.py', "blue"]
        subprocess.Popen(ros_command)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        rospy.logerr("Usage: send_goal_node <goal_latitude> <goal_longitude>")
        sys.exit(1)

    goal_latitude = float(sys.argv[1])
    goal_longitude = float(sys.argv[2])

    try:
        MoveBaseGoalSender(goal_latitude, goal_longitude)
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted")
