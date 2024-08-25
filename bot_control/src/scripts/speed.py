#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(goal_list):
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # Wait until the action server has started up and started listening for goals.
    client.wait_for_server()

    for goal in goal_list:
        # Creates a new goal with the MoveBaseGoal constructor
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose.position.x = goal['x']
        move_base_goal.target_pose.pose.position.y = goal['y']
        move_base_goal.target_pose.pose.orientation.w = goal['orientation']

        # Sends the goal to the action server.
        client.send_goal(move_base_goal)
        # Waits for the server to finish performing the action.
        wait = client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        else:
            # Result of executing the action
            rospy.loginfo("Reached point: (x: {}, y: {})".format(goal['x'], goal['y']))

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')

        # Define the list of goals (points) with position and orientation
        goals = [
            {'x': 0.0, 'y': 0.0, 'orientation': 0.0},
            {'x': 0.0, 'y': 0.0, 'orientation': 0.0},
            {'x': 0.0, 'y': 0.0, 'orientation': 0.0}
        ]

        movebase_client(goals)
        rospy.loginfo("All goals execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
