#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
from std_msgs.msg import Header


class StopSignBehavior:
    def __init__(self):
        rospy.init_node('stop_sign_behavior_node')

        self.car_cmd_pub = rospy.Publisher("/birdie/car_cmd", Twist2DStamped, queue_size=1)
        self.stop_sign_sub = rospy.Subscriber("/birdie/stop_sign_detected", BoolStamped, self.stop_sign_cb)
        self.lane_follow_sub = rospy.Subscriber("/birdie/lane_controller/car_cmd", Twist2DStamped, self.lane_follow_cb)

        # State management
        self.state = "LANE_FOLLOWING"
        self.ignore_stop_signs = False
        self.stopping_timer_started = False

        # Parameters
        self.ignore_duration = 4  # seconds to ignore stop signs after stopping
        self.wait_duration = 3    # seconds to stop at stop sign

    def stop_sign_cb(self, msg):
        if self.ignore_stop_signs:
            return

        if msg.data and self.state == "LANE_FOLLOWING":
            rospy.loginfo("Stop Sign Detected!")
            self.state = "STOPPING"
            self.stopping_timer_started = False

    def lane_follow_cb(self, msg):
        if self.state == "LANE_FOLLOWING":
            self.car_cmd_pub.publish(msg)

        elif self.state == "STOPPING":
            self.publish_stop()
            if not self.stopping_timer_started:
                rospy.loginfo("Stopping for stop sign...")
                rospy.Timer(rospy.Duration(self.wait_duration), self.resume_after_stop, oneshot=True)
                self.stopping_timer_started = True
                self.state = "WAITING"

        elif self.state == "WAITING":
            self.publish_stop()

        elif self.state == "IGNORING_TAGS":
            self.car_cmd_pub.publish(msg)

    def publish_stop(self):
        stop_msg = Twist2DStamped()
        stop_msg.header = Header()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.v = 0.0
        stop_msg.omega = 0.0
        self.car_cmd_pub.publish(stop_msg)

    def resume_after_stop(self, event):
        rospy.loginfo("Stop complete. Ignoring stop signs for a short duration.")
        self.ignore_stop_signs = True
        self.state = "IGNORING_TAGS"
        rospy.Timer(rospy.Duration(self.ignore_duration), self.stop_ignoring_tags, oneshot=True)

    def stop_ignoring_tags(self, event):
        rospy.loginfo("Resuming normal stop sign detection.")
        self.ignore_stop_signs = False
        self.state = "LANE_FOLLOWING"

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    StopSignBehavior().run()

