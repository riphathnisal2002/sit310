#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, AprilTagDetectionArray
from std_msgs.msg import Header

class Autopilot:
    def __init__(self):
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        self.stop_in_progress = False  # To avoid retriggering during stop behavior

        self.cmd_vel_pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/birdie/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/birdie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        rospy.on_shutdown(self.clean_shutdown)
        rospy.spin()

    def tag_callback(self, msg):
        if self.stop_in_progress or self.robot_state != "LANE_FOLLOWING":
            return

        # Check if a Stop Sign is detected (AprilTag ID 1 is typically used for stop signs)
        for detection in msg.detections:
            if detection.tag_id == 1:
                rospy.loginfo("Stop sign detected. Executing stop behavior...")
                self.stop_in_progress = True
                self.execute_stop_behavior()
                return

    def execute_stop_behavior(self):
        # Step 1: Stop the robot
        self.set_state("NORMAL_JOYSTICK_CONTROL")
        self.stop_robot()
        rospy.sleep(3)  # Wait for 3 seconds

        # Step 2: Move forward in open-loop mode to get past the stop sign
        rospy.loginfo("Moving forward to clear stop sign from view...")
        forward_cmd = Twist2DStamped()
        forward_cmd.header.stamp = rospy.Time.now()
        forward_cmd.v = 0.15  # Forward speed
        forward_cmd.omega = 0.0
        self.cmd_vel_pub.publish(forward_cmd)
        rospy.sleep(1.0)  # Move forward for 1 second

        # Step 3: Stop and resume lane following
        rospy.loginfo("Resuming lane following...")
        self.stop_robot()
        self.set_state("LANE_FOLLOWING")
        self.stop_in_progress = False

    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def set_state(self, state):
        self.robot_state = state
        state_msg = FSMState()
        state_msg.header = Header()
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
