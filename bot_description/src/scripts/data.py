#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
import math

def euler_to_quaternion(roll, pitch, yaw):
    """ Convert Euler angles (in degrees) to quaternion """
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    qx = math.sin(roll / 2.0) * math.cos(pitch / 2.0) * math.cos(yaw / 2.0) - math.cos(roll / 2.0) * math.sin(pitch / 2.0) * math.sin(yaw / 2.0)
    qy = math.cos(roll / 2.0) * math.sin(pitch / 2.0) * math.cos(yaw / 2.0) + math.sin(roll / 2.0) * math.cos(pitch / 2.0) * math.sin(yaw / 2.0)
    qz = math.cos(roll / 2.0) * math.cos(pitch / 2.0) * math.sin(yaw / 2.0) - math.sin(roll / 2.0) * math.sin(pitch / 2.0) * math.cos(yaw / 2.0)
    qw = math.cos(roll / 2.0) * math.cos(pitch / 2.0) * math.cos(yaw / 2.0) + math.sin(roll / 2.0) * math.sin(pitch / 2.0) * math.sin(yaw / 2.0)
    
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator')
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        # Define waypoints (x, y, theta)
        self.waypoints = [
            (0.0, 0.0, 0.0),
            (5.0, 0.0, 45.0),
            (5.0, 5.0, 90.0),
            (0.0, 5.0, 0.0),
            (0.0, 0.0, 135.0),
        ]

    def navigate(self):
        for point in self.waypoints:
            x, y, theta = point
            waypoint = Pose(Point(x, y, 0.0), euler_to_quaternion(0.0, 0.0, theta))
            self.goto_waypoint(waypoint)
            rospy.sleep(1)

    def goto_waypoint(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = waypoint

        self.client.send_goal(goal)
        self.client.wait_for_result()
        
        if self.client.get_result():
            rospy.loginfo("Reached waypoint: " + str(waypoint))
        else:
            rospy.logwarn("Failed to reach waypoint: " + str(waypoint))

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        navigator.navigate()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
