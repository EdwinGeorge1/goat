#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def main():
    # Initialize the ROS node
    rospy.init_node('order_publisher', anonymous=True)
    
    # Create a publisher object
    pub = rospy.Publisher('order_received', String, queue_size=10)
    
    # Wait for the publisher to connect
    rospy.sleep(1)
    
    # Set the table number
    table_number = 'table1'  # Change this to the table you want to test
    
    # Log info and publish the message
    rospy.loginfo(f"Publishing order for {table_number}")
    pub.publish(table_number)
    
    # Sleep to ensure the message is sent
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Order publisher interrupted.")

