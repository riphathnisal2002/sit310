#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        # Initialize class variables
        self.last_distance = 0
        self.start_distance = 0
        self.goal_distance = 0
        self.dist_goal_active = False
        self.forward_movement = True

        self.goal_angle = 0
        self.start_angle = 0
        self.angle_goal_active = False

        self.pose = Pose()

        self.position_goal = None
        self.position_goal_active = False
        self.stage = "rotate"  # or "move"

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Subscribers
        rospy.Subscriber("/turtle_dist", Float64, self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/goal_position", Point, self.goal_position_callback)
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)

        # Publisher
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Timer callback
        rospy.Timer(rospy.Duration(0.01), self.timer_callback)

        rospy.loginfo("Initialized node!")

        rospy.spin()

    def pose_callback(self, msg):
        self.pose = msg

    def distance_callback(self, msg):
        self.last_distance = msg.data

    def goal_distance_callback(self, msg):
        self.goal_distance = abs(msg.data)
        self.start_distance = self.last_distance
        self.dist_goal_active = True
        self.forward_movement = msg.data >= 0
        self.angle_goal_active = False
        self.position_goal_active = False

    def goal_angle_callback(self, msg):
        self.goal_angle = abs(msg.data)
        self.start_angle = self.pose.theta
        self.angle_direction = 1 if msg.data >= 0 else -1
        self.angle_goal_active = True
        self.dist_goal_active = False
        self.position_goal_active = False

    def goal_position_callback(self, msg):
        self.position_goal = msg
        self.position_goal_active = True
        self.dist_goal_active = False
        self.angle_goal_active = False
        self.stage = "rotate"

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def compute_distance_to_goal(self):
        dx = self.position_goal.x - self.pose.x
        dy = self.position_goal.y - self.pose.y
        return math.sqrt(dx**2 + dy**2)

    def compute_angle_to_goal(self):
        dx = self.position_goal.x - self.pose.x
        dy = self.position_goal.y - self.pose.y
        return math.atan2(dy, dx)

    def timer_callback(self, event):
        cmd = Twist()

        if self.dist_goal_active:
            if abs(self.last_distance - self.start_distance) < self.goal_distance:
                cmd.linear.x = 1.0 if self.forward_movement else -1.0
            else:
                self.dist_goal_active = False
                rospy.loginfo("Reached distance goal")

        elif self.angle_goal_active:
            current = self.pose.theta
            delta_angle = self.normalize_angle(current - self.start_angle)
            if abs(delta_angle) < self.goal_angle:
                cmd.angular.z = 1.0 * self.angle_direction
            else:
                self.angle_goal_active = False
                rospy.loginfo("Reached rotation goal")

        elif self.position_goal_active:
            angle_to_target = self.compute_angle_to_goal()
            angle_diff = self.normalize_angle(angle_to_target - self.pose.theta)
            distance_to_target = self.compute_distance_to_goal()

            if self.stage == "rotate":
                if abs(angle_diff) > 0.05:
                    cmd.angular.z = 1.0 if angle_diff > 0 else -1.0
                else:
                    self.stage = "move"
            elif self.stage == "move":
                if distance_to_target > 0.1:
                    cmd.linear.x = 1.0
                else:
                    self.position_goal_active = False
                    rospy.loginfo("Reached position goal")

        self.velocity_publisher.publish(cmd)

if __name__ == '__main__':
    try:
        TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException:
        pass
