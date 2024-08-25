#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations as tft

def yaw_to_quaternion(yaw):
    """
    Convert a yaw angle in degrees to a quaternion.
    """
    # Convert degrees to radians
    yaw_rad = yaw * (3.14159265359 / 180.0)
    # Convert yaw to quaternion
    quaternion = tft.quaternion_from_euler(0, 0, yaw_rad)
    return quaternion

def movebase_client(goals):
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # Wait until the action server has started up and started listening for goals.
    client.wait_for_server()

    for goal in goals:
        # Create a new goal with the MoveBaseGoal constructor
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose.position.x = goal['x']
        move_base_goal.target_pose.pose.position.y = goal['y']
        
        # Convert yaw angle to quaternion
        quaternion = yaw_to_quaternion(goal['yaw'])
        move_base_goal.target_pose.pose.orientation.x = quaternion[0]
        move_base_goal.target_pose.pose.orientation.y = quaternion[1]
        move_base_goal.target_pose.pose.orientation.z = quaternion[2]
        move_base_goal.target_pose.pose.orientation.w = quaternion[3]

        # Send the goal to the action server.
        client.send_goal(move_base_goal)
        # Wait for the server to finish performing the action.
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        else:
            rospy.loginfo("Reached point: (x: {}, y: {}, yaw: {})".format(goal['x'], goal['y'], goal['yaw']))

# If the python node is executed as the main process (sourced directly)
if __name__ == '__main__':
    try:
        # Initialize a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('waypoint_navigation_client')

        # Define the list of goals to navigate with specified yaw angles
        goals = [
            {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            {'x': 5.0, 'y': 0.0, 'yaw': 45.0},
            {'x': 5.0, 'y': 5.0, 'yaw': 90.0},
            {'x': 0.0, 'y': 5.0, 'yaw': 0.0},
            {'x': 0.0, 'y': 0.0, 'yaw': 135.0}
        ]

        movebase_client(goals)
        rospy.loginfo("All waypoints reached!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint navigation interrupted.")

