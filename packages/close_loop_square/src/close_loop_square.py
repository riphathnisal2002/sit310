#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped, FSMState

class ClosedLoopSquare:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        rospy.init_node('closed_loop_square_node', anonymous=True)
        rospy.loginfo("Node initialized")

        self.pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/birdie/right_wheel_encoder_node/tick', WheelEncoderStamped, self.right_encoder_callback)
        rospy.Subscriber('/birdie/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)
        rospy.Subscriber('/birdie/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)

        self.ticks_per_meter = 345
        self.ticks_per_90_deg = 100  # Adjust based on testing

        self.linear_speed = 0.5
        self.angular_speed = 0.5

        self.break_time = rospy.Duration(1.0)
        self.break_start = None

        self.right_ticks = 0
        self.left_ticks = 0
        self.tasks = []
        self.current_task = None
        self.is_running = False

        for i in range(4):
            self.tasks.append({'action': 'move', 'ticks': self.ticks_per_meter})
            self.tasks.append({'action': 'turn', 'ticks': self.ticks_per_90_deg})

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
            for i in range(4):
                self.tasks.append({'action': 'move', 'ticks': self.ticks_per_meter})
                self.tasks.append({'action': 'turn', 'ticks': self.ticks_per_90_deg})

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def right_encoder_callback(self, msg):
        self.right_ticks = msg.data

    def left_encoder_callback(self, msg):
        self.left_ticks = msg.data

    def move_distance(self, ticks, start_left, start_right):
        delta_left = self.left_ticks - start_left
        delta_right = self.right_ticks - start_right
        avg_ticks = (delta_left + delta_right) / 2.0

        if abs(avg_ticks) >= abs(ticks):
            rospy.loginfo("Move complete")
            self.stop_robot()
            return True

        direction = 1 if ticks > 0 else -1
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = self.linear_speed * direction
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        return False

    def rotate_in_place(self, ticks, start_left, start_right):
        delta_left = self.left_ticks - start_left
        delta_right = self.right_ticks - start_right
        delta_rotation = delta_right - delta_left

        if abs(delta_rotation) >= abs(ticks):
            rospy.loginfo("Turn complete")
            self.stop_robot()
            return True

        direction = 1 if ticks > 0 else -1
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = self.angular_speed * direction
        self.pub.publish(self.cmd_msg)
        return False

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
            self.current_task['start_left'] = self.left_ticks
            self.current_task['start_right'] = self.right_ticks
            rospy.loginfo(f"Starting {self.current_task['action']}")

        task_complete = False
        if self.current_task['action'] == 'move':
            task_complete = self.move_distance(
                self.current_task['ticks'],
                self.current_task['start_left'],
                self.current_task['start_right']
            )
        elif self.current_task['action'] == 'turn':
            task_complete = self.rotate_in_place(
                self.current_task['ticks'],
                self.current_task['start_left'],
                self.current_task['start_right']
            )

        if task_complete:
            self.current_task = None
            self.stop_robot()
            if self.tasks:
                self.break_start = rospy.Time.now()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        closed_loop_square = ClosedLoopSquare()
        closed_loop_square.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in main: {str(e)}")

