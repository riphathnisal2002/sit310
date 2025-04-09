#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float64
import math

class DistanceCalculator:
    def __init__(self):
        rospy.init_node('distance_turtle', anonymous=True)

        # Publisher to publish the total distance
        self.distance_pub = rospy.Publisher('/turtle_distance', Float64, queue_size=10)

        # Subscriber to the turtle's pose
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        # Variables to track the previous position and total distance
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0

        rospy.loginfo("distance_turtle node started.")
        rospy.spin()

    def pose_callback(self, msg):
        current_x = msg.x
        current_y = msg.y

        if self.prev_x is not None and self.prev_y is not None:
            dx = current_x - self.prev_x
            dy = current_y - self.prev_y
            distance = math.sqrt(dx**2 + dy**2)
            self.total_distance += distance

            # Publish the updated distance
            self.distance_pub.publish(Float64(self.total_distance))
            rospy.loginfo(f"Distance Traveled: {self.total_distance:.2f}")

        # Update previous position
        self.prev_x = current_x
        self.prev_y = current_y

if __name__ == '__main__':
    try:
        DistanceCalculator()
    except rospy.ROSInterruptException:
        pass
