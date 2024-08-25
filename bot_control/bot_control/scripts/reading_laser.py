#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    # Print the length of the ranges array
    print("Number of laser scan points:", len(msg.ranges))
    # Print the range value at the center of the laser scan
    center_index = len(msg.ranges) // 2
    print("Range value at the center of the laser scan:", msg.ranges[center_index])

    # Modify the laser scan data to have a range of 0 to 120 degrees
    filtered_ranges = msg.ranges[:120]
    filtered_msg = LaserScan()
    filtered_msg.header = msg.header
    filtered_msg.angle_min = msg.angle_min
    filtered_msg.angle_max = msg.angle_min + (120 * msg.angle_increment)
    filtered_msg.angle_increment = msg.angle_increment
    filtered_msg.time_increment = msg.time_increment
    filtered_msg.scan_time = msg.scan_time
    filtered_msg.range_min = msg.range_min
    filtered_msg.range_max = msg.range_max
    filtered_msg.ranges = filtered_ranges

    # Publish the filtered laser scan data to /filtered_scan topic
    filtered_scan_publisher.publish(filtered_msg)

def main():
    rospy.init_node('laser_scan_filter')
    rospy.Subscriber('/scan', LaserScan, callback)
    global filtered_scan_publisher
    filtered_scan_publisher = rospy.Publisher('/filtered_scan', LaserScan, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    main()
