#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Autopilot:
    def __init__(self):
        rospy.init_node('autopilot_node', anonymous=True)
        self.robot_state = "LANE_FOLLOWING"
        self.tag_cooldown = rospy.Duration(5.0)  # 5-second cooldown
        self.last_tag_time = rospy.Time(0)

        rospy.on_shutdown(self.clean_shutdown)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/birdie/fsm_node/mode', FSMState, queue_size=1)

        # Subscribers
        rospy.Subscriber('/birdie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        rospy.spin()

    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return

        current_time = rospy.Time.now()

        if len(msg.detections) > 0 and (current_time - self.last_tag_time) > self.tag_cooldown:
            self.last_tag_time = current_time
            rospy.loginfo("AprilTag detected. Stopping for 3 seconds.")
            self.stop_and_wait()

    def stop_and_wait(self):
        # Stop the robot
        self.stop_robot()

        # Switch FSM mode to something that doesn't conflict with lane following
        self.set_state("NORMAL_JOYSTICK_CONTROL")

        # Wait for 3 seconds
        rospy.sleep(3.0)

        # Resume lane following
        self.set_state("LANE_FOLLOWING")
        rospy.loginfo("Resuming lane following.")

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

    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
