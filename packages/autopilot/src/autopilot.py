#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Autopilot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        # Current robot FSM state
        self.robot_state = "LANE_FOLLOWING"

        # Clean shutdown behavior
        rospy.on_shutdown(self.clean_shutdown)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/birdie/fsm_node/mode', FSMState, queue_size=1)

        # Subscribers
        rospy.Subscriber('/birdie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)

        rospy.spin()

    def tag_callback(self, msg):
        # Only act if currently in lane following mode
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

        rospy.loginfo("AprilTag detected. Stopping for 3 seconds.")

        # Stop the robot
        self.stop_robot()

        # Switch FSM to stop lane following
        self.set_state("NORMAL_JOYSTICK_CONTROL")

        # Sleep for a few seconds (blocking is okay here)
        rospy.sleep(3.0)

        # Resume lane following
        self.set_state("LANE_FOLLOWING")

        rospy.loginfo("Resuming lane following.")

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
