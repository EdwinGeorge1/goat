import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator', anonymous=True)
        self.waypoints = [(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]  # Define your waypoints here
        self.current_waypoint_index = 0
        self.current_waypoint = self.waypoints[self.current_waypoint_index]
        self.robot_pose = None
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)

    def odometry_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def navigate_to_waypoint(self):
        if self.robot_pose is not None:
            robot_x = self.robot_pose.position.x
            robot_y = self.robot_pose.position.y
            target_x, target_y = self.current_waypoint

            distance_to_target = sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)

            if distance_to_target < 0.1:  # Define a threshold distance for reaching the waypoint
                rospy.loginfo("Reached waypoint: {}".format(self.current_waypoint))
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.waypoints):
                    self.current_waypoint = self.waypoints[self.current_waypoint_index]
                else:
                    rospy.loginfo("Reached all waypoints")
                    return

            # Calculate the angle to the target waypoint
            angle_to_target = atan2(target_y - robot_y, target_x - robot_x)
            
            # Define the linear and angular velocities
            linear_vel = 0.1  # Adjust as needed
            angular_vel = 0.5  # Adjust as needed

            # Create Twist message
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = linear_vel
            cmd_vel_msg.angular.z = angular_vel

            # Publish Twist message
            self.cmd_vel_pub.publish(cmd_vel_msg)

    def run(self):
        rate = rospy.Rate(10)  # Adjust the loop rate as needed
        while not rospy.is_shutdown():
            self.navigate_to_waypoint()
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass

