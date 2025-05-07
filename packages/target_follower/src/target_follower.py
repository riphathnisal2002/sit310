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

        self.shutting_down = False  # Shutdown flag

        # Control parameters
        self.Kp = 4.5           # Proportional gain
        self.omega_min = 5.0    # Minimum omega to overcome friction
        self.omega_max = 12.0   # Maximum allowed omega

        rospy.spin()

    def tag_callback(self, msg):
        if self.shutting_down:
            return  # Prevent callback actions during shutdown
        self.move_robot(msg.detections)

    def clean_shutdown(self):
        rospy.loginfo("Shutting down. Stopping robot...")
        self.shutting_down = True
        self.stop_robot()

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        try:
            self.cmd_vel_pub.publish(cmd_msg)
        except rospy.ROSException:
            rospy.logwarn("Failed to publish stop command: publisher is closed.")

    def move_robot(self, detections):
        if self.shutting_down:
            return  # Avoid publishing if node is shutting down

        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()

        if len(detections) == 0:
            rospy.loginfo("No tag detected. Seeking...")
            cmd_msg.v = 0.0
            cmd_msg.omega = 6.5  # Rotate to find the tag
            self.cmd_vel_pub.publish(cmd_msg)
            return

        transform = detections[0].transform.translation
        x = transform.x
        y = transform.y
        z = transform.z

        rospy.loginfo("Tracking tag. x: %.3f, y: %.3f, z: %.3f", x, y, z)

        # Proportional controller
        omega = self.Kp * x

        if abs(omega) < self.omega_min and abs(x) > 0.01:
            omega = self.omega_min * (1 if omega > 0 else -1)

        omega = max(-self.omega_max, min(self.omega_max, omega))

        cmd_msg.v = 0.0
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass

