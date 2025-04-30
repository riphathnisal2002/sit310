#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
import math

class ClosedLoopController:
    def __init__(self):
        rospy.init_node("closed_loop_controller")

        # === Constants (adjust for your robot)
        self.TICKS_PER_REV = 360
        self.WHEEL_RADIUS = 0.03  # meters
        self.WHEEL_BASE = 0.2     # meters

        self.left_encoder = 0
        self.right_encoder = 0
        self.left_encoder_start = 0
        self.right_encoder_start = 0

        # === Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/left_encoder", Int32, self.left_encoder_callback)
        rospy.Subscriber("/right_encoder", Int32, self.right_encoder_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        rospy.sleep(1)  # Wait for everything to initialize

    # === Encoder Callbacks
    def left_encoder_callback(self, msg):
        self.left_encoder = msg.data

    def right_encoder_callback(self, msg):
        self.right_encoder = msg.data

    def reset_encoders(self):
        self.left_encoder_start = self.left_encoder
        self.right_encoder_start = self.right_encoder

    def ticks_to_distance(self, ticks):
        return 2 * math.pi * self.WHEEL_RADIUS * ticks / self.TICKS_PER_REV

    def get_distance_moved(self):
        left_ticks = self.left_encoder - self.left_encoder_start
        right_ticks = self.right_encoder - self.right_encoder_start
        avg_ticks = (left_ticks + right_ticks) / 2.0
        return self.ticks_to_distance(avg_ticks)

    def get_rotation_moved(self):
        left_ticks = self.left_encoder - self.left_encoder_start
        right_ticks = self.right_encoder - self.right_encoder_start
        delta_ticks = right_ticks - left_ticks
        return self.ticks_to_distance(delta_ticks) / self.WHEEL_BASE  # radians

    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        self.publish_velocity(0, 0)

    # === Movement Functions
    def move_distance(self, distance_m, speed_mps):
        rospy.loginfo(f"Moving {distance_m:.2f} m at {speed_mps:.2f} m/s")
        self.reset_encoders()
        direction = 1 if distance_m >= 0 else -1
        self.publish_velocity(speed_mps * direction, 0)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            moved = self.get_distance_moved()
            if abs(moved) >= abs(distance_m):
                break
            rate.sleep()
        self.stop()

    def rotate_angle(self, angle_rad, angular_speed_rps):
        rospy.loginfo(f"Rotating {math.degrees(angle_rad):.1f}° at {math.degrees(angular_speed_rps):.1f}°/s")
        self.reset_encoders()
        direction = 1 if angle_rad >= 0 else -1
        self.publish_velocity(0, angular_speed_rps * direction)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rotated = self.get_rotation_moved()
            if abs(rotated) >= abs(angle_rad):
                break
            rate.sleep()
        self.stop()

    def draw_square(self, side_length=1.0):
        rospy.loginfo("Drawing square...")
        for _ in range(4):
            self.move_distance(side_length, 0.2)
            rospy.sleep(0.5)
            self.rotate_angle(math.pi / 2, math.radians(30))

    # === Joystick Callback
    def joy_callback(self, msg):
        # Map buttons (adjust if needed)
        A = msg.buttons[0]
        B = msg.buttons[1]
        X = msg.buttons[2]
        Y = msg.buttons[3]
        LB = msg.buttons[4]

        if A:
            self.move_distance(1.0, 0.2)
        elif B:
            self.move_distance(-1.0, 0.2)
        elif X:
            self.rotate_angle(-math.pi / 2, math.radians(30))
        elif Y:
            self.rotate_angle(math.pi / 2, math.radians(30))
        elif LB:
            self.draw_square()

if __name__ == "__main__":
    try:
        ClosedLoopController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
