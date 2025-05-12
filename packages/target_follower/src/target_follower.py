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
        self.rotation_speed = 3.0
        self.forward_speed = 0.2
        self.rotation_threshold = 0.035  # x-axis threshold to decide rotation
        self.desired_distance = 0.4      # meters
        self.distance_tolerance = 0.05   # acceptable range around desired distance

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

        if len(detections) == 0:
            rospy.loginfo("No tag detected. Staying stationary.")
            cmd_msg.v = 0.0
            cmd_msg.omega = 0.0
            self.cmd_vel_pub.publish(cmd_msg)
            return

        # --- Tag detected ---
        closest_tag = min(detections, key=lambda tag: tag.transform.translation.z)
        x = closest_tag.transform.translation.x
        z = closest_tag.transform.translation.z
        rospy.loginfo("Tag detected. x: %.3f, z: %.3f", x, z)

        # --- Rotation control ---
        if x > self.rotation_threshold:
            cmd_msg.omega = -abs(self.rotation_speed)  # Turn right
            rospy.loginfo("Rotating right")
        elif x < -self.rotation_threshold:
            cmd_msg.omega = abs(self.rotation_speed)   # Turn left
            rospy.loginfo("Rotating left")
        else:
            cmd_msg.omega = 0.0  # Aligned
            rospy.loginfo("Tag aligned")

        # --- Forward/backward movement control ---
        if z > self.desired_distance + self.distance_tolerance:
            cmd_msg.v = self.forward_speed  # Move forward
            rospy.loginfo("Moving forward")
        elif z < self.desired_distance - self.distance_tolerance:
            cmd_msg.v = -self.forward_speed  # Too close, move back
            rospy.loginfo("Too close! Moving backward")
        else:
            cmd_msg.v = 0.0  # Within acceptable range
            rospy.loginfo("At optimal distance - stopping")

        self.cmd_vel_pub.publish(cmd_msg)


if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
