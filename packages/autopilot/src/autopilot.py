#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class TagStopper:
    def __init__(self):
        rospy.init_node('tag_stopper_node', anonymous=True)
        self.cmd_pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/birdie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback)

        self.last_tag_time = rospy.Time(0)
        self.tag_cooldown = rospy.Duration(5.0)

        rospy.on_shutdown(self.stop_robot)
        rospy.spin()

    def tag_callback(self, msg):
        now = rospy.Time.now()
        if len(msg.detections) > 0 and (now - self.last_tag_time > self.tag_cooldown):
            rospy.loginfo("Tag detected! Sending stop command.")
            self.last_tag_time = now
            self.stop_robot()
            rospy.sleep(3.0)
            rospy.loginfo("Stop complete, resuming control.")

    def stop_robot(self):
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = 0.0
        cmd.omega = 0.0
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        TagStopper()
    except rospy.ROSInterruptException:
        pass
