#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray
import time

class Autopilot:
    def __init__(self):
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        self.last_stop_time = None
        self.stop_cooldown = 5.0  # Seconds

        rospy.on_shutdown(self.clean_shutdown)

        self.cmd_vel_pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/birdie/fsm_node/mode', FSMState, queue_size=1)

        rospy.Subscriber('/birdie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        rospy.spin()

    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return

        current_time = time.time()

        # Only react to stop sign (AprilTag ID 31)
        stop_sign_detected = any(tag.id == 31 for tag in msg.detections)

        if not stop_sign_detected:
            return

        # Avoid multiple stops in short time
        if self.last_stop_time and (current_time - self.last_stop_time < self.stop_cooldown):
            return

        self.last_stop_time = current_time
        self.perform_stop_and_resume()

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
        state_msg.state = state
        self.state_pub.publish(state_msg)

    def perform_stop_and_resume(self):
        rospy.loginfo("Stop sign (ID 31) detected. Stopping for 3 seconds.")

        self.stop_robot()
        self.set_state("NORMAL_JOYSTICK_CONTROL")

        rospy.sleep(3.0)

        self.set_state("LANE_FOLLOWING")
        rospy.loginfo("Resuming lane following.")

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass

