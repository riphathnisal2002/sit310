#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        self.last_pose = Pose()
        self.last_distance = 0.0

        # Distance goal
        self.goal_distance = 0.0
        self.start_distance = 0.0
        self.dist_goal_active = False
        self.forward_movement = True

        # Angle goal
        self.angle_goal_active = False
        self.start_angle = 0.0
        self.target_angle = 0.0

        # Position goal
        self.position_goal_active = False
        self.target_x = 0.0
        self.target_y = 0.0

        # Subscribers
        rospy.Subscriber("/turtle_dist", Float64, self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/goal_position", Point, self.goal_position_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        # Publisher
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        rospy.Timer(rospy.Duration(0.01), self.timer_callback)
        rospy.loginfo("Initialized node!")

        rospy.spin()

    def pose_callback(self, msg):
        self.last_pose = msg

    def distance_callback(self, msg):
        self.last_distance = msg.data

    def goal_angle_callback(self, msg):
        if msg.data == 0:
            return
        self.start_angle = self.last_pose.theta
        self.target_angle = self.normalize_angle(self.last_pose.theta + msg.data)
        self.angle_goal_active = True
        rospy.loginfo(f"Rotating to angle: {self.target_angle:.2f}")

    def goal_distance_callback(self, msg):
        if msg.data == 0:
            return
        self.start_distance = self.last_distance
        self.goal_distance = abs(msg.data)
        self.forward_movement = msg.data > 0
        self.dist_goal_active = True
        rospy.loginfo(f"Moving {'forward' if self.forward_movement else 'backward'} for {self.goal_distance:.2f} units")

    def goal_position_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.position_goal_active = True
        rospy.loginfo(f"Moving to position: ({self.target_x:.2f}, {self.target_y:.2f})")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def timer_callback(self, event):
        twist = Twist()

        if self.angle_goal_active:
            angle_error = self.normalize_angle(self.target_angle - self.last_pose.theta)
            if abs(angle_error) < 0.01:
                twist.angular.z = 0
                self.angle_goal_active = False
                rospy.loginfo("Finished rotating.")
            else:
                twist.angular.z = 1.5 * angle_error

        elif self.dist_goal_active:
            traveled = abs(self.last_distance - self.start_distance)
            if traveled >= self.goal_distance:
                twist.linear.x = 0
                self.dist_goal_active = False
                rospy.loginfo("Finished moving straight.")
            else:
                twist.linear.x = 1.5 if self.forward_movement else -1.5

        elif self.position_goal_active:
            dx = self.target_x - self.last_pose.x
            dy = self.target_y - self.last_pose.y
            distance_to_goal = math.sqrt(dx**2 + dy**2)

            if distance_to_goal < 0.1:
                self.position_goal_active = False
                twist.linear.x = 0
                twist.angular.z = 0
                rospy.loginfo("Reached position goal.")
            else:
                desired_angle = math.atan2(dy, dx)
                angle_error = self.normalize_angle(desired_angle - self.last_pose.theta)

                if abs(angle_error) > 0.1:
                    twist.angular.z = 2.0 * angle_error
                else:
                    twist.linear.x = 2.0

        self.velocity_publisher.publish(twist)

if __name__ == '__main__':
    try:
        TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException:
        pass
