#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, FSMState

class RotateFourTasks:
    def __init__(self):
        rospy.init_node('rotate_four_tasks_node', anonymous=True)
        rospy.loginfo("Node initialized")

        self.cmd_msg = Twist2DStamped()
        self.pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/birdie/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)
        rospy.Subscriber('/birdie/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/birdie/fsm_node/mode', FSMState, self.fsm_callback)

        # Ticks calibration (adjust based on your robot)
        self.ticks_per_90_deg = 500
        self.break_time = rospy.Duration(1.0)
        self.break_start = None

        self.left_ticks = 0
        self.right_ticks = 0

        self.tasks = []
        self.current_task = None
        self.is_running = False

        rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def fsm_callback(self, msg):
        if msg.state == "LANE_FOLLOWING":
            rospy.loginfo("FSM: Starting CW/CCW rotation sequences")
            self.is_running = True
            self.tasks = []
            self.current_task = None
            self.break_start = None

            # Set 2 speeds (change values as needed)
            speed1 = 3.0
            speed2 = 7.0

            # Add 4 tasks: CW and CCW at two speeds
            self.add_rotation_task(360, speed1)   # CW at speed1
            self.add_rotation_task(-360, speed1)  # CCW at speed1
            self.add_rotation_task(360, speed2)   # CW at speed2
            self.add_rotation_task(-360, speed2)  # CCW at speed2

        else:
            rospy.loginfo("FSM: Stopping motion")
            self.is_running = False
            self.stop_robot()

    def add_rotation_task(self, angle_deg, angular_speed):
        ticks = abs(int((angle_deg / 90.0) * self.ticks_per_90_deg))
        direction = 1 if angle_deg > 0 else -1
        self.tasks.append({
            'action': 'turn_ticks',
            'ticks': ticks,
            'speed': abs(angular_speed) * direction
        })

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def timer_callback(self, event):
        if not self.is_running:
            return

        if self.break_start is not None:
            if rospy.Time.now() - self.break_start < self.break_time:
                return
            self.break_start = None

        if self.current_task is None:
            if not self.tasks:
                rospy.loginfo("All rotation tasks completed")
                self.is_running = False
                self.stop_robot()
                return

            self.current_task = self.tasks.pop(0)
            self.current_task['start_left'] = self.left_ticks
            self.current_task['start_right'] = self.right_ticks
            rospy.loginfo(f"Starting rotation task: ω = {self.current_task['speed']} rad/s")

        task_done = self.turn_ticks()
        if task_done:
            rospy.loginfo("Rotation task complete")
            self.current_task = None
            self.stop_robot()
            if self.tasks:
                self.break_start = rospy.Time.now()

    def turn_ticks(self):
        moved_left = abs(self.left_ticks - self.current_task['start_left'])
        moved_right = abs(self.right_ticks - self.current_task['start_right'])
        avg_moved = (moved_left + moved_right) / 2.0

        rospy.loginfo(f"Rotating... Avg ticks: {avg_moved}/{self.current_task['ticks']}")

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
        node = RotateFourTasks()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unhandled error: {str(e)}")

