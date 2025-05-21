#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Autopilot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"

        # Shutdown callback
        rospy.on_shutdown(self.clean_shutdown)

        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/birdie/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/birdie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        rospy.spin()  # Listen for messages indefinitely

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return

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

    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    def move_robot(self, detections):
        if len(detections) == 0:
            return

        # Switch to joystick control to override lane following
        self.set_state("NORMAL_JOYSTICK_CONTROL")

        # Create velocity command message
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()

        # React to first recognized tag only
        for detection in detections:
            tag_id = detection.id[0]
            rospy.loginfo(f"Detected AprilTag ID: {tag_id}")

            if tag_id == 1:
                # Move forward
                cmd_msg.v = 0.3
                cmd_msg.omega = 0.0
            elif tag_id == 2:
                # Turn left
                cmd_msg.v = 0.0
                cmd_msg.omega = 1.0
            elif tag_id == 3:
                # Turn right
                cmd_msg.v = 0.0
                cmd_msg.omega = -1.0
            else:
                rospy.loginfo("Unrecognized tag ID. Ignoring.")
                return

            self.cmd_vel_pub.publish(cmd_msg)
            rospy.sleep(1.0)  # Move for 1 second
            self.stop_robot()
            rospy.sleep(0.5)
            break  # Only act on one detection at a time

        # Return to lane following mode
        self.set_state("LANE_FOLLOWING")

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
