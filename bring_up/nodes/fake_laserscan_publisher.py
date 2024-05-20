#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import random

# Publish unkown space
# def generate_fake_scan():
#     scan = LaserScan()
#     scan.header.stamp = rospy.Time.now()
#     scan.header.frame_id = 'lidar_link'
#     scan.angle_min = -3.14  # -180 degrees
#     scan.angle_max = 3.14   # 180 degrees
#     scan.angle_increment = 0.01  # Approx. 0.57 degrees per step
#     scan.time_increment = 0.0
#     scan.scan_time = 0.1
#     scan.range_min = 0.2
#     scan.range_max = 10.0

#     num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
#     scan.ranges = [scan.range_max for _ in range(num_readings)]
#     scan.intensities = [1.0 for _ in range(num_readings)]

#     return scan

# Publish free space
def generate_fake_scan():
    scan = LaserScan()
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = 'lidar_link'
    scan.angle_min = -3.14  # -180 degrees
    scan.angle_max = 3.14   # 180 degrees
    scan.angle_increment = 0.01  # Approx. 0.57 degrees per step
    scan.time_increment = 0.0
    scan.scan_time = 0.1
    scan.range_min = 0.2
    scan.range_max = 10.0

    num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
    # Set all ranges to the maximum range, indicating free space
    scan.ranges = [scan.range_max for _ in range(num_readings)]
    # Set intensities to 0.0 for all readings
    scan.intensities = [0.0 for _ in range(num_readings)]

    return scan


def scan_publisher():
    rospy.init_node('scan_publisher_node', anonymous=True)
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        fake_scan = generate_fake_scan()
        pub.publish(fake_scan)
        rate.sleep()

if __name__ == '__main__':
    try:
        scan_publisher()
    except rospy.ROSInterruptException:
        pass
