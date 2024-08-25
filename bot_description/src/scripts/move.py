#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import tf

# Waypoints: [x (meters), y (meters), yaw_angle(degrees)]
waypoints = [
    [0, 0, 0],
    [5, 0, 45],
    [5, 5, 90],
    [0, 5, 0],
    [0, 0, 135]
]

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('move_waypoints')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.pose = Pose()
        self.rate = rospy.Rate(10)  # 10 Hz

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def move_to_waypoint(self, waypoint):
        goal_x, goal_y, goal_yaw_deg = waypoint
        goal_yaw = math.radians(goal_yaw_deg)

        while not rospy.is_shutdown():
            position = self.pose.position
            orientation = self.pose.orientation

            # Convert orientation from quaternion to euler angles
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]

            # Calculate the distance and angle to the waypoint
            distance = math.sqrt((goal_x - position.x) ** 2 + (goal_y - position.y) ** 2)
            angle_to_goal = math.atan2(goal_y - position.y, goal_x - position.x)

            # Proportional control for rotation
            angle_diff = angle_to_goal - yaw

            # Normalize the angle difference
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            # Move towards the waypoint
            twist = Twist()

            if abs(angle_diff) > 0.1:
                twist.angular.z = 0.5 * angle_diff
            else:
                twist.angular.z = 0.0

            if distance > 0.1:
                twist.linear.x = 0.5 * distance
            else:
                twist.linear.x = 0.0

            # Rotate to the final desired yaw
            if distance < 0.1 and abs(goal_yaw - yaw) > 0.1:
                twist.angular.z = 0.5 * (goal_yaw - yaw)
            elif distance < 0.1 and abs(goal_yaw - yaw) <= 0.1:
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                break

            self.cmd_pub.publish(twist)
            self.rate.sleep()

    def navigate(self):
        for waypoint in waypoints:
            self.move_to_waypoint(waypoint)
        rospy.loginfo("Navigation complete.")

if __name__ == '__main__':
    navigator = WaypointNavigator()
    navigator.navigate()

