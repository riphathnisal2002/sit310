#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('target_follower_node', anonymous=True)

        # Clean shutdown behavior
        rospy.on_shutdown(self.clean_shutdown)

        # Publisher: sends velocity commands
        self.cmd_vel_pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

        # Subscriber: receives AprilTag detections
        rospy.Subscriber('/birdie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        rospy.spin()

    # Callback for AprilTag detections
    def tag_callback(self, msg):
        self.move_robot(msg.detections)

    # Clean shutdown: stop the robot
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Publish zero velocity to stop the robot
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    # Main control logic
    def move_robot(self, detections):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0  # Always stationary (no forward/backward motion)

        # --- Seek Mode: No tag detected ---
        if len(detections) == 0:
            rospy.loginfo("No tag detected. Seeking...")
            cmd_msg.omega = 0.5  # Constant rotation speed
            self.cmd_vel_pub.publish(cmd_msg)
            return

        # --- Look at Object Mode: Tag detected ---
        # Get the first detection's position
        x = detections[0].transform.translation.x  # lateral offset
        z = detections[0].transform.translation.z  # distance to tag

        rospy.loginfo("Tag detected. x: %.3f, z: %.3f", x, z)

        # --- Proportional Controller ---
        k_p = 2.0         # Gain
        min_omega = 0.2   # Minimum omega to overcome friction
        max_omega = 1.0   # Maximum allowed omega

        error = x  # x > 0 => tag is to the right; x < 0 => tag is to the left
        omega = k_p * error

        # Apply minimum threshold
        if abs(omega) < min_omega:
            omega = min_omega * (1 if omega > 0 else -1)

        # Apply maximum limit
        if abs(omega) > max_omega:
            omega = max_omega * (1 if omega > 0 else -1)

        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)

        # Optional: slow down command rate
        rospy.sleep(0.05)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
