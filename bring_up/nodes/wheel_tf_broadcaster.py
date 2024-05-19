#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import JointState
import math

class WheelTfBroadcaster:
    def __init__(self):
        rospy.init_node('wheel_tf_broadcaster')
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        
        # Wheel positions relative to base_link (from your URDF)
        self.wheel_positions = {
            'base_to_flwheel': (0.386, 0.375, 0),
            'base_to_blwheel': (-0.45, 0.375, 0),
            'base_to_middle_wheel': (-0.037, 0.375, 0),
            'base_to_frwheel': (0.40, -0.352, 0),
            'base_to_brwheel': (-0.45, -0.352, 0),
            'base_to_middle_wheel_2': (-0.037, -0.352, 0)
        }

    def joint_states_callback(self, msg):
        joint_positions = dict(zip(msg.name, msg.position))

        for joint, position in joint_positions.items():
            if joint in self.wheel_positions:
                self.broadcast_transform('base_link', joint.replace('base_to_', ''), self.wheel_positions[joint], position)
        
    def broadcast_transform(self, parent_frame, child_frame, translation, rotation_angle):
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame

        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]

        # Convert rotation angle to quaternion
        quat = self.euler_to_quaternion(rotation_angle, 0, 0) # Rotating around x-axis
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        self.broadcaster.sendTransform(transform)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return (qx, qy, qz, qw)

if __name__ == '__main__':
    try:
        node = WheelTfBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
