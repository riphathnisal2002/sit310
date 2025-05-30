#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState
from sensor_msgs.msg import Range  # Importing the Range message for ToF
import time

class Autopilot:
    def __init__(self):
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        self.tof_distance = float('inf')

        self.stop_start_time = None
        self.waiting_for_clear = False
        self.overtake_in_progress = False
        self.tof_threshold = 0.4  # in meters

        rospy.on_shutdown(self.clean_shutdown)

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/birdie/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/birdie/front_center_tof_driver_node/range', Range, self.tof_callback)

        # Run main loop
        self.main_timer = rospy.Timer(rospy.Duration(0.1), self.main_loop)  # 10Hz loop

        rospy.spin()

    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    def tof_callback(self, msg):
        self.tof_distance = msg.range

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

    def main_loop(self, event):
        if self.overtake_in_progress:
            return  # Let the overtake finish

        if self.tof_distance < self.tof_threshold:
            if not self.waiting_for_clear:
                rospy.loginfo("Object detected! Stopping and waiting...")
                self.stop_robot()
                self.set_state("NORMAL_JOYSTICK_CONTROL")
                self.stop_start_time = time.time()
                self.waiting_for_clear = True
            else:
                elapsed = time.time() - self.stop_start_time
                if elapsed >= 5.0:
                    rospy.loginfo("Waited 5 seconds, starting overtake...")
                    self.overtake()
        else:
            if self.waiting_for_clear:
                rospy.loginfo("Path cleared, resuming lane following.")
                self.set_state("LANE_FOLLOWING")
                self.waiting_for_clear = False

            # Lane following default behavior
            self.publish_forward_command()

    def publish_forward_command(self, speed=0.2):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = speed
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

    def overtake(self):
        self.overtake_in_progress = True
        self.set_state("NORMAL_JOYSTICK_CONTROL")

        # Improved open-loop overtake behavior
        self.rotate_in_place(omega=2.0, duration=1.5)  # Larger left turn
        self.move_forward(duration=2.5)                # Go around the obstacle
        self.rotate_in_place(omega=-2.0, duration=1.5) # Return to lane
        self.move_forward(duration=2.0)                # Straighten out

        rospy.loginfo("Overtake complete. Resuming lane following.")
        self.set_state("LANE_FOLLOWING")
        self.overtake_in_progress = False
        self.waiting_for_clear = False

    def rotate_in_place(self, omega=2.0, duration=1.5):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(duration)
        self.stop_robot()

    def move_forward(self, speed=0.2, duration=2.0):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = speed
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.sleep(duration)
        self.stop_robot()

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass

