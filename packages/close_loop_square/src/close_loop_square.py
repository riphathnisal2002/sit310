#!/usr/bin/env python3

import rospy
import math
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped, FSMState
from sensor_msgs.msg import Range  # TOF sensor message

class ClosedLoopSquare:
    def __init__(self):
        rospy.init_node('closed_loop_square_node', anonymous=True)
        rospy.loginfo("Node initialized")

        # Encoder tick counts
        self.left_ticks = 0
        self.right_ticks = 0

        # Control message
        self.cmd_msg = Twist2DStamped()

        # Publishers & subscribers
        self.pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/birdie/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback)
        rospy.Subscriber('/birdie/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/birdie/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/birdie/tof_driver_node/range', Range, self.tof_callback)

        # Calibration constants (adjust these)
        self.ticks_per_meter = 545
        self.ticks_per_90_deg = 50

        # Speeds
        self.linear_speed = 0.5
        self.angular_speed = 8.5

        # Task control
        self.tasks = []
        self.current_task = None
        self.is_running = False
        self.break_time = rospy.Duration(1.0)
        self.break_start = None

        # TOF sensor
        self.tof_distance = float('inf')
        self.tof_threshold = 0.25  # meters (25 cm)

        # Main loop
        rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def tof_callback(self, msg):
        self.tof_distance = msg.range
        rospy.loginfo_throttle(1.0, f"[TOF] Distance: {self.tof_distance:.2f} m")

    def fsm_callback(self, msg):
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            rospy.loginfo("FSM: Switching to joystick mode")
            self.is_running = False
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            rospy.loginfo("FSM: Starting square pattern")
            self.is_running = True
            self.current_task = None
            self.break_start = None
            self.tasks = []

            # Add 4 sides and 4 90° turns
            side_length = 0.5  # meters
            for _ in range(4):
                self.add_move_task(side_length)
                self.add_turn_task(90)

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def encoder_callback(self, msg):
        self.right_ticks = msg.data

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def timer_callback(self, event):
        if not self.is_running:
            return

        # TOF check
        if 0.0 < self.tof_distance < self.tof_threshold:
            rospy.logwarn("⚠️ Obstacle within 25cm. Stopping...")
            self.stop_robot()
            return

        # Wait between tasks
        if self.break_start is not None:
            if (rospy.Time.now() - self.break_start) < self.break_time:
                return
            self.break_start = None

        # Start new task
        if self.current_task is None:
            if not self.tasks:
                rospy.loginfo("✅ Completed square pattern!")
                self.is_running = False
                return

            self.current_task = self.tasks.pop(0)
            self.cmd_msg = Twist2DStamped()  # Reset cmd message
            if self.current_task['action'] == 'move':
                self.current_task['start'] = self.right_ticks
            elif self.current_task['action'] == 'turn_ticks':
                self.current_task['start_left'] = self.left_ticks
                self.current_task['start_right'] = self.right_ticks

            rospy.loginfo(f"➡️ Starting task: {self.current_task['action']}")

        # Execute current task
        task_complete = False
        if self.current_task['action'] == 'move':
            task_complete = self.move_ticks()
        elif self.current_task['action'] == 'turn_ticks':
            task_complete = self.turn_ticks()

        if task_complete:
            rospy.loginfo("✅ Task complete")
            self.stop_robot()
            self.current_task = None
            if self.tasks:
                self.break_start = rospy.Time.now()

    def add_move_task(self, distance, speed=None):
        if speed is None:
            speed = self.linear_speed
        ticks = abs(int(distance * self.ticks_per_meter))
        direction = 1 if distance >= 0 else -1
        self.tasks.append({
            'action': 'move',
            'ticks': ticks,
            'speed': abs(speed) * direction
        })

    def add_turn_task(self, angle_deg, angular_speed=None):
        if angular_speed is None:
            angular_speed = self.angular_speed
        ticks = abs(int((angle_deg / 90.0) * self.ticks_per_90_deg))
        direction = 1 if angle_deg >= 0 else -1
        self.tasks.append({
            'action': 'turn_ticks',
            'ticks': ticks,
            'speed': abs(angular_speed) * direction
        })

    def move_ticks(self):
        moved_ticks = abs(self.right_ticks - self.current_task['start'])
        if moved_ticks >= self.current_task['ticks']:
            return True
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = self.current_task['speed']
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        return False

    def turn_ticks(self):
        moved_left = abs(self.left_ticks - self.current_task['start_left'])
        moved_right = abs(self.right_ticks - self.current_task['start_right'])
        avg_moved = (moved_left + moved_right) / 2.0

        rospy.loginfo_throttle(0.5, f"🔁 Turning... Left: {moved_left}, Right: {moved_right}, Target: {self.current_task['ticks']}")

        if avg_moved >= self.current_task['ticks']:
            return True
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = self.current_task['speed']
        self.pub.publish(self.cmd_msg)
        return False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        closed_loop_square = ClosedLoopSquare()
        rospy.sleep(2.0)  # Optional startup delay
        closed_loop_square.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in main: {str(e)}")

