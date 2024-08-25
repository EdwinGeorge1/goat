#!/usr/bin/env python

import rospy
import actionlib
import tf  # Import tf here
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator', anonymous=True)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.waypoints = [
            self.create_pose(1.0, 1.0, 0.0),
            self.create_pose(2.0, 1.0, 0.0),
            self.create_pose(2.0, 2.0, 1.57),
            self.create_pose(1.0, 2.0, 3.14)
        ]

        self.navigate_waypoints()

    def create_pose(self, x, y, yaw):
        pose = Pose()
        pose.position = Point(x, y, 0)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.orientation = Quaternion(*quaternion)
        return pose

    def navigate_waypoints(self):
        for i, waypoint in enumerate(self.waypoints):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = waypoint

            rospy.loginfo(f"Navigating to waypoint {i + 1}")
            self.client.send_goal(goal)
            self.client.wait_for_result()

            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Reached waypoint {i + 1}")
            else:
                rospy.loginfo(f"Failed to reach waypoint {i + 1}")
                break

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint navigation interrupted.")

