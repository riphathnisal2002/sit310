#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, AprilTagDetectionArray

class Autopilot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"

        # Flag to ignore tags while waiting/moving past a Stop sign
        self.ignore_tags = False

        # Time to stop at each stop sign
        self.stop_duration = rospy.get_param("~stop_duration", 3.0)

        # Time to ignore tags after stopping
        self.ignore_duration = rospy.get_param("~ignore_duration", 2.0)

        # When shutdown signal is received, run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)

        # Init publishers and subscribers (replace 'jeff' with your robot name if needed)
        self.cmd_vel_pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/birdie/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/birdie/apriltag_detector_node/detections', AprilTagDetectionArray,
                         self.tag_callback, queue_size=1)

        rospy.spin()  # Keep node running and listening for callbacks

    # Callback for AprilTag detection
    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING" or self.ignore_tags:
            return
        self.move_robot(msg.detections)

    # Clean shutdown procedure
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Stop robot by publishing zero velocity
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    # Change the robot FSM state
    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.state = self.robot_state
        self.state_pub.publish(state_msg)

    # Logic for handling detected AprilTags
    def move_robot(self, detections):
        if not detections:
            return

        # Filter for stop sign tags with tag_id = 31
        stop_sign_detections = [d for d in detections if d.tag_id == 31]

        if not stop_sign_detections:
            return

        rospy.loginfo("Stop sign (tag_id=31) detected → stopping for {:.1f}s".format(self.stop_duration))

        # Stop the robot
        self.set_state("NORMAL_JOYSTICK_CONTROL")
        self.stop_robot()

        # Pause for stop duration
        rospy.sleep(self.stop_duration)

        # Ignore additional stop tags for a short time
        self.ignore_tags = True
        rospy.Timer(rospy.Duration(self.ignore_duration), self.reset_ignore, oneshot=True)

        # Resume lane following
        self.set_state("LANE_FOLLOWING")
        rospy.loginfo("Resuming lane following")

    # Re-enable tag detection
    def reset_ignore(self, event):
        self.ignore_tags = False
        rospy.loginfo("AprilTag detections re-enabled")


if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
