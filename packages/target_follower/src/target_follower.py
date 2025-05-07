#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        self.cmd_vel_pub = rospy.Publisher(
            '/birdie/car_cmd_switch_node/cmd',
            Twist2DStamped,
            queue_size=1
        )

        rospy.Subscriber(
            '/birdie/apriltag_detector_node/detections',
            AprilTagDetectionArray,
            self.tag_callback,
            queue_size=1
        )

        # Hysteresis control parameters
        self.omega_fixed = 8.0      # Fixed turn rate
        self.dead_zone = 0.05       # Neutral zone for no movement

        rospy.spin()

    def tag_callback(self, msg):
        self.move_robot(msg.detections)

    def clean_shutdown(self):
        rospy.loginfo("Shutting down. Stopping robot...")
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

        # Seek Mode: No tag detected
        if len(detections) == 0:
            rospy.loginfo("No tag detected. Seeking...")
            cmd_msg.v = 0.0
            cmd_msg.omega = self.omega_fixed  # Spin in place
            self.cmd_vel_pub.publish(cmd_msg)
            return

        # Tag detected
        x = detections[0].transform.translation.x
        rospy.loginfo("Tracking tag. x: %.3f", x)

        # Hysteresis control: turn left or right if outside dead zone
        if x > self.dead_zone:
            cmd_msg.omega = self.omega_fixed
        elif x < -self.dead_zone:
            cmd_msg.omega = -self.omega_fixed
        else:
            cmd_msg.omega = 0.0  # In the center, stop rotating

        cmd_msg.v = 0.0  # No forward motion
        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        Target_Follower()
    except rospy.ROSInterruptException:
        pass
