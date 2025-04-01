#!/usr/bin/env python3

# Import Dependencies
import rospy
from geometry_msgs.msg import Twist
import time

def move_turtle_square():
    rospy.init_node('turtlesim_square_node', anonymous=True)

    # Init publisher
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Turtles are great at drawing squares!")

    # Create a Twist message for controlling the turtle
    cmd_vel_msg = Twist()

    while not rospy.is_shutdown():
        for _ in range(4):  # Loop to draw 4 sides of a square
            # Move forward for some time
            cmd_vel_msg.linear.x = 2.0  # Linear velocity
            cmd_vel_msg.angular.z = 0.0  # No angular velocity (straight)
            velocity_publisher.publish(cmd_vel_msg)  # Publish the forward command
            rospy.sleep(2)  # Adjust this time based on how far the turtle should move

            # Turn 90 degrees
            cmd_vel_msg.linear.x = 0.0  # Stop moving forward
            cmd_vel_msg.angular.z = 1.57  # Set angular velocity to 90 degrees per second (1.57 radians per second)
            velocity_publisher.publish(cmd_vel_msg)  # Publish the turn command
            rospy.sleep(1.5)  # Time to turn 90 degrees (adjust based on angular speed)

            # Stop the turtle after completing each movement cycle
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            velocity_publisher.publish(cmd_vel_msg)  # Stop moving
            rospy.sleep(0.5)  # Pause before starting the next side

if __name__ == '__main__':
    try:
        move_turtle_square()
    except rospy.ROSInterruptException:
        pass

