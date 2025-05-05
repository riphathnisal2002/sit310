#!/usr/bin/env python3
import sys
import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped
from sensor_msgs.msg import Range # 1️⃣ Import the Range message

class ClosedLoopSquare:
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "tesla")

        # Encoder state
        self.right_ticks = 0

        # ToF sensor state
        self.current_range = float('inf')
        self.obstacle_detected = False
        self.tof_threshold = 0.3 # metres: stop if object closer than this

        # Encoder calibration
        self.ticks_per_meter = 630
        self.ticks_per_90_degrees = 45

        rospy.init_node("closed_loop_square_node", anonymous=True)

        # Publishers & Subscribers
        self.pub = rospy.Publisher(
            f"/{self.robot_name}/car_cmd_switch_node/cmd",
            Twist2DStamped, queue_size=1
        )
        rospy.Subscriber(f"/{self.robot_name}/right_wheel_encoder_node/tick",
                         WheelEncoderStamped, self.right_encoder_cb, queue_size=1)
        rospy.Subscriber(f"/{self.robot_name}/fsm_node/mode",
                         FSMState, self.fsm_cb, queue_size=1)
        # 2️⃣ Subscribe to the front-center ToF range topic
        rospy.Subscriber(f"/{self.robot_name}/front_center_tof_driver_node/range",
                         Range, self.tof_cb, queue_size=1)

        # Reusable cmd
        self.cmd_msg = Twist2DStamped()

    # ───── Callbacks ───────────────────────────────────────────────────────
    def right_encoder_cb(self, msg):
        self.right_ticks = msg.data

    def fsm_cb(self, msg):
        if msg.state == "LANE_FOLLOWING":
            rospy.sleep(1.0)
            self.draw_square()

    # 3️⃣ ToF callback: set flag when range < threshold
    def tof_cb(self, msg):
        self.current_range = msg.range
        self.obstacle_detected = (msg.range < self.tof_threshold)

    # ────── Helpers ─────────────────────────────────────────────────────────
    def _stop(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    # ────── Primitives ──────────────────────────────────────────────────────
    def move_straight(self, distance_m, v=0.5):
        direction = 1 if distance_m >= 0 else -1
        goal_ticks = abs(distance_m) * self.ticks_per_meter
        start_ticks = self.right_ticks

        rate = rospy.Rate(50)
        while abs(self.right_ticks - start_ticks) < goal_ticks and not rospy.is_shutdown():
            # 4️⃣ If obstacle too close: stop & wait
            if self.obstacle_detected:
                rospy.logwarn(f"Obstacle at {self.current_range:.2f} m – stopping")
                self._stop()
                # wait loop
                while self.obstacle_detected and not rospy.is_shutdown():
                    rate.sleep()
                rospy.loginfo("Path clear – resuming")
                # continue driving toward goal, ticks still counted
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = v * direction
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rate.sleep()

        self._stop()

    def rotate_in_place(self, angle_deg, omega=1.0):
        direction = 1 if angle_deg >= 0 else -1
        goal_ticks = abs(angle_deg) / 90.0 * self.ticks_per_90_degrees
        start_ticks = self.right_ticks

        rate = rospy.Rate(50)
        while abs(self.right_ticks - start_ticks) < goal_ticks and not rospy.is_shutdown():
            if self.obstacle_detected:
                rospy.logwarn(f"Obstacle at {self.current_range:.2f} m during rotation – stopping")
                self._stop()
                while self.obstacle_detected and not rospy.is_shutdown():
                    rate.sleep()
                rospy.loginfo("Path clear – resuming rotation")
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.0
            self.cmd_msg.omega = omega * direction
            self.pub.publish(self.cmd_msg)
            rate.sleep()

        self._stop()

    # ────── Demo ────────────────────────────────────────────────────────────
    def draw_square(self):
        rospy.loginfo("Starting closed-loop square…")
        for _ in range(4):
            self.move_straight(+1.0, v=0.41)
            rospy.sleep(0.3)
            self.rotate_in_place(-90.0, omega=3.0)
            rospy.sleep(0.3)
        rospy.loginfo("Square finished ✔")

    # ────── Run ─────────────────────────────────────────────────────────────
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        ClosedLoopSquare().run()
    except rospy.ROSInterruptException:
        pass
