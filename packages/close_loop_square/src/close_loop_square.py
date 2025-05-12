#!/usr/bin/env python3

import rospy
import math
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped, FSMState

class ClosedLoopSquare:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        rospy.init_node('closed_loop_square_node', anonymous=True)
        rospy.loginfo("Node initialized")
        
        self.left_ticks = 0
        self.right_ticks = 0

        self.pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/birdie/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback)
        rospy.Subscriber('/birdie/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/birdie/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        
        # Calibration constants
        self.ticks_per_meter = 545
        self.ticks_per_90_deg = 50  # UPDATED

        # Default control speeds
        self.linear_speed = 0.5
        self.angular_speed = 8.5

        # Timing control
        self.break_time = rospy.Duration(1.0)
        self.break_start = None

        self.tasks = []

        self.current_task = None
        self.is_running = False

        rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def fsm_callback(self, msg):
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            rospy.loginfo("Switching to joystick control")
            self.is_running = False
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            rospy.loginfo("Starting square pattern")
            self.is_running = True
            self.current_task = None
            self.break_start = None
            self.tasks = []

            side_length = 0.5  # meters
            
            self.add_turn_task(360, angular_speed=5)
            self.add_wait_task(2.0)  # wait 2 seconds
            self.add_turn_task(360, angular_speed=10)

            
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

        if self.break_start is not None:
            if (rospy.Time.now() - self.break_start) < self.break_time:
                return
            self.break_start = None

        if self.current_task is None:
            if not self.tasks:
                rospy.loginfo("Square pattern completed!")
                self.is_running = False
                return

            self.current_task = self.tasks.pop(0)
            self.current_task['start'] = self.right_ticks
            rospy.loginfo(f"Starting task: {self.current_task['action']}")

        task_complete = False
        if self.current_task['action'] == 'move':
            task_complete = self.move_ticks()
        elif self.current_task['action'] == 'turn_ticks':
            if 'start_left' not in self.current_task or 'start_right' not in self.current_task:
                self.current_task['start_left'] = self.left_ticks
                self.current_task['start_right'] = self.right_ticks
            task_complete = self.turn_ticks()

        elif self.current_task['action'] == 'wait':
            if self.current_task['start_time'] is None:
                self.current_task['start_time'] = rospy.Time.now()
                rospy.loginfo(f"Waiting for {self.current_task['duration'].to_sec()} seconds")

            if rospy.Time.now() - self.current_task['start_time'] >= self.current_task['duration']:
                task_complete = True

        if task_complete:
            self.current_task = None
            self.stop_robot()
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
        avg_moved = (moved_left + moved_right) / 2

        rospy.loginfo(f"Turning... Left: {moved_left}, Right: {moved_right}, Avg: {avg_moved}, Target: {self.current_task['ticks']}")

        if avg_moved >= self.current_task['ticks']:
            return True
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = self.current_task['speed']
        self.pub.publish(self.cmd_msg)
        return False

    def run(self):
        rospy.spin()

    def add_wait_task(self, duration):
        self.tasks.append({
            'action': 'wait',
            'duration': rospy.Duration(duration),
            'start_time': None
        })


if __name__ == '__main__':
    try:
        closed_loop_square = ClosedLoopSquare()
        rospy.sleep(2.0)
        closed_loop_square.is_running = True
        closed_loop_square.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in main: {str(e)}")
