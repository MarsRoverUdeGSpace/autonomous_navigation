#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import NavSatFix
import utm

class MoveBaseGoalSender:
    def __init__(self):

        rospy.init_node('send_goal_node')
        rospy.loginfo("send_goal_node started.")

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

        # Wait until we get the initial GPS coordinates
        rospy.loginfo("Waiting for initial GPS coordinates...")
        while self.initial_lat is None or self.initial_lon is None:
            rospy.sleep(0.1)

        # Example GPS coordinates (use at least 5 floating points)
        lat1, lon1 = 20.657628333333335, -103.32630833333333
        lat2, lon2 = self.initial_lat, self.initial_lon

        rospy.loginfo("Received goal GPS coordinates: lat={}, lon={}".format(lat2, lon2))

        # Convert initial and goal GPS coordinates to UTM coordinates
        utm_x1, utm_y1, zone_number1, zone_letter1 = self.gps_to_utm(lat1, lon1)
        utm_x2, utm_y2, zone_number2, zone_letter2 = self.gps_to_utm(lat2, lon2)

        rospy.loginfo("Initial UTM: x={}, y={}, zone={}{}".format(utm_x1, utm_y1, zone_number1, zone_letter1))
        rospy.loginfo("Goal UTM: x={}, y={}, zone={}{}".format(utm_x2, utm_y2, zone_number2, zone_letter2))

        # Ensure the UTM coordinates are in the same zone
        if zone_number1 != zone_number2 or zone_letter1 != zone_letter2:
            rospy.logerr("Initial and goal coordinates are in different UTM zones")
        else:
            # Calculate the relative position
            #! Esto no va
            # rel_x = utm_x2 - utm_x1
            # rel_y = utm_y2 - utm_y1

            # Send the goal to move_base and publish the markers for RViz
            self.send_goal_and_publish_markers(0, 0, utm_x2, utm_y2)

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
        initial_marker_pub = rospy.Publisher('initial_pose_marker', PoseStamped, queue_size=10)
        goal_marker_pub = rospy.Publisher('goal_pose_marker', PoseStamped, queue_size=10)

        initial_marker = PoseStamped()
        initial_marker.header.frame_id = "map"
        initial_marker.header.stamp = rospy.Time.now()
        initial_marker.pose = Pose(Point(initial_x, initial_y, 0), Quaternion(0, 0, z, w))

        goal_marker = PoseStamped()
        goal_marker.header.frame_id = "map"
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.pose = Pose(Point(goal_x, goal_y, 0), Quaternion(0, 0, z, w))

        # Wait for the publishers to connect
        while initial_marker_pub.get_num_connections() < 1 or goal_marker_pub.get_num_connections() < 1:
            rospy.sleep(0.1)

        initial_marker_pub.publish(initial_marker)
        rospy.loginfo("Published initial marker at: x={}, y={}".format(initial_x, initial_y))

        goal_marker_pub.publish(goal_marker)
        rospy.loginfo("Published goal marker at: x={}, y={}".format(goal_x, goal_y))

        # Create a MoveBaseGoal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(goal_x, goal_y, 0), Quaternion(0, 0, z, w))

        # Send the goal to move_base
        rospy.loginfo("Sending goal: x={}, y={}".format(goal_x, goal_y))
        client.send_goal(goal)
        client.wait_for_result()

        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached successfully")
        else:
            rospy.logwarn("Failed to reach the goal")


if __name__ == "__main__":
    try:
        MoveBaseGoalSender()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted")