#!/usr/bin/env python3

import rospy
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
import math

class RobotNavigator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('robot_navigator')

        # Initialize the action client for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        if not self.client.wait_for_server(rospy.Duration(30)):
            rospy.logerr("move_base action server not available!")
            rospy.signal_shutdown("move_base action server not available!")
            return
        rospy.loginfo("Connected to move_base action server.")

        # Load waypoints from the parameter server
        if not rospy.has_param('/waypoints'):
            rospy.logerr("No 'waypoints' parameter found on the parameter server.")
            rospy.signal_shutdown("Waypoints not available.")
            return

        self.waypoints = rospy.get_param('/waypoints')
        if not isinstance(self.waypoints, dict):
            rospy.logerr("'waypoints' parameter should be a dictionary.")
            rospy.signal_shutdown("Invalid waypoints format.")
            return

        rospy.loginfo(f"Loaded waypoints: {self.waypoints}")

        # Subscribe to the 'order_received' topic
        rospy.Subscriber('order_received', String, self.handle_order)

    def move_to_waypoint(self, waypoint_name):
        rospy.loginfo(f"Attempting to move to waypoint: {waypoint_name}")

        # Check if the waypoint exists
        if waypoint_name not in self.waypoints:
            rospy.logwarn(f"Waypoint '{waypoint_name}' not found!")
            return False

        # Retrieve waypoint coordinates
        waypoint = self.waypoints[waypoint_name]
        if not (isinstance(waypoint, list) and len(waypoint) == 3):
            rospy.logwarn(f"Waypoint '{waypoint_name}' has invalid format. Expected [x, y, theta].")
            return False

        x, y, theta = waypoint
        rospy.loginfo(f"Moving to coordinates: x={x}, y={y}, theta={theta}")

        # Create a MoveBaseGoal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation = Quaternion(*quaternion)

        # Send the goal to move_base
        rospy.loginfo(f"Sending goal to move_base: {goal}")
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)

        # Wait for the result with a timeout
        wait_duration = rospy.Duration(60)  # Adjust as needed
        finished_before_timeout = self.client.wait_for_result(wait_duration)

        if not finished_before_timeout:
            rospy.logwarn("move_base action did not finish before the timeout.")
            self.client.cancel_goal()
            return False
        else:
            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Successfully reached waypoint '{waypoint_name}'.")
                return True
            else:
                rospy.logwarn(f"Failed to reach waypoint '{waypoint_name}'. State: {state}")
                return False

    def feedback_callback(self, feedback):
        rospy.logdebug(f"Received feedback: {feedback}")

    def handle_order(self, msg):
        table_number = msg.data.strip().lower()
        rospy.loginfo(f"Order received for: {table_number}")

        # Define the sequence of waypoints to navigate
        sequence = ['kitchen', table_number, 'home']

        for waypoint in sequence:
            rospy.loginfo(f"Navigating to '{waypoint}'...")
            success = self.move_to_waypoint(waypoint)
            if not success:
                rospy.logerr(f"Failed to navigate to '{waypoint}'. Aborting task.")
                return
            rospy.loginfo(f"Arrived at '{waypoint}'.")

        rospy.loginfo("Task completed successfully.")

if __name__ == '__main__':
    try:
        navigator = RobotNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node interrupted.")

