#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        self.cmd_vel_pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/birdie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        # Control parameters
        self.Kp = 4.5       # Proportional gain (tune this)
        self.omega_min = 8.0  # Minimum angular speed (to overcome friction)
        self.omega_max = 12.0  # Maximum angular speed

        rospy.spin()

    def tag_callback(self, msg):
        self.move_robot(msg.detections)

    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def move_robot(self, detections):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()

        # --- Seek Mode ---
        if len(detections) == 0:
            rospy.loginfo("No tag detected. Seeking...")
            cmd_msg.v = 0.0
            cmd_msg.omega = 8.5  # Rotate in place to search
            self.cmd_vel_pub.publish(cmd_msg)
            return

        # --- Tag Detected: Rotate in Place using P-Control ---
        transform = detections[0].transform.translation
        x = transform.x  # left-right offset
        z = transform.z  # depth
        y = transform.y  # vertical offset

        rospy.loginfo("Tag found. x: %.3f, y: %.3f, z: %.3f", x, y, z)

        # Calculate angular velocity
        omega = self.Kp * x

        # Apply minimum threshold to overcome friction
        if abs(omega) < self.omega_min and abs(x) > 0.01:
            omega = self.omega_min * (1 if omega > 0 else -1)

        # Clamp to maximum
        omega = max(-self.omega_max, min(self.omega_max, omega))

        cmd_msg.v = 0.0            # No linear motion
        cmd_msg.omega = omega      # Angular motion only
        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
