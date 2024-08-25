#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

def euler_to_quaternion(yaw):
    # Convert yaw (in degrees) to radians
    yaw_rad = yaw * (3.14159 / 180.0)
    # Convert Euler angles to quaternion
    quaternion = quaternion_from_euler(0, 0, yaw_rad)  # roll and pitch are zero
    return quaternion

def movebase_client(goals):
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # Wait until the action server has started up and started listening for goals.
    client.wait_for_server()

    for i, goal in enumerate(goals):
        # Create a new goal with the MoveBaseGoal constructor
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose.position.x = goal['x']
        move_base_goal.target_pose.pose.position.y = goal['y']
        
        # Convert orientation (yaw angle) to quaternion
        quaternion = euler_to_quaternion(goal['orientation'])
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
            rospy.loginfo("Reached point: (x: {}, y: {}, orientation: {})".format(goal['x'], goal['y'], goal['orientation']))

        # Check if this is the last waypoint
        if i == len(goals) - 1:
            # Send spinning goal after the last waypoint
            spin_on_spot(client, 3)  # 3 full rotations (clockwise)

def spin_on_spot(client, num_rotations):
    # Create a new goal to spin in place
    move_base_goal = MoveBaseGoal()
    move_base_goal.target_pose.header.frame_id = "map"
    move_base_goal.target_pose.header.stamp = rospy.Time.now()
    move_base_goal.target_pose.pose.position.x = 0.0
    move_base_goal.target_pose.pose.position.y = 0.0
    
    # Calculate the total rotation in degrees (360° per rotation)
    total_rotation = num_rotations * 360.0
    # Set the orientation to spin in a clockwise direction
    quaternion = euler_to_quaternion(total_rotation)
    move_base_goal.target_pose.pose.orientation.x = quaternion[0]
    move_base_goal.target_pose.pose.orientation.y = quaternion[1]
    move_base_goal.target_pose.pose.orientation.z = quaternion[2]
    move_base_goal.target_pose.pose.orientation.w = quaternion[3]

    # Send the spinning goal to the action server
    client.send_goal(move_base_goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available for spinning!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Completed spinning on the spot.")

# If the python node is executed as the main process (sourced directly)
if __name__ == '__main__':
    try:
        # Initialize a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('waypoint_navigation_client')

        # Define the list of goals with positions and orientations
        goals = [
            {'x': 0.0, 'y': 0.0, 'orientation': 0.0},    # Point (0,0) with 0° orientation
            {'x': 5.0, 'y': 0.0, 'orientation': 45.0},   # Point (5,0) with 45° orientation
            {'x': 5.0, 'y': 5.0, 'orientation': 90.0},   # Point (5,5) with 90° orientation
            {'x': 0.0, 'y': 5.0, 'orientation': 0.0},    # Point (0,5) with 0° orientation
            {'x': 0.0, 'y': 0.0, 'orientation': 135.0}   # Point (0,0) with 135° orientation
        ]

        movebase_client(goals)
        rospy.loginfo("All waypoints reached!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Waypoint navigation interrupted.")

