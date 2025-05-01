#!/usr/bin/env python3

import rospy 
import math
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped, FSMState

class ClosedLoopSquare:
    def __init__(self):
        # Initialize message
        self.cmd_msg = Twist2DStamped()

        # Initialize ROS node
        rospy.init_node('closed_loop_square_node', anonymous=True)
        rospy.loginfo("Node initialized")
        
        self.left_ticks = 0

        
        # Initialize publishers/subscribers
        self.pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/birdie/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback)
        rospy.Subscriber('/birdie/left_wheel_encoder_node/tick', WheelEncoderStamped, self.left_encoder_callback)

        rospy.Subscriber('/birdie/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        
        # Set the parameters
        self.ticks_per_meter = 345
        self.ticks_per_90_deg = 100

        # Set the control speeds
        self.linear_speed = 0.5
        self.angular_speed = 0.5
        
        # Task timing
        self.break_time = rospy.Duration(1.0)
        self.break_start = None
        
        # Initialize encoders and tasks
        self.right_ticks = 0
        self.tasks = []
        self.current_task = None
        self.is_running = False

        # Initialize timer
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

    def fsm_callback(self, msg):
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            rospy.loginfo("Switching to joystick control")
            self.is_running = False
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":            
            rospy.loginfo("Starting square pattern")
            # Always reset the task queue and state when entering LANE_FOLLOWING
            self.is_running = True
            self.current_task = None
            self.break_start = None
            
            # Reset task queue
            self.tasks = []
            move_sequence = [
                (1, 0.3),
                (-1, 0.3),
                (1, 0.6),
                (-1, 0.6),
            ]

            for direction, distance in move_sequence:
                ticks = int(direction * self.ticks_per_meter * distance)
                self.tasks.append({
                    'action': 'move',
                    'ticks': ticks
                })
 
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
            
        # Handle breaks between tasks
        if self.break_start is not None:
            if (rospy.Time.now() - self.break_start) < self.break_time:
                return  # Still in break, do nothing
            self.break_start = None  # Break is over
        
        # Start new task if needed
        if self.current_task is None:
            if not self.tasks:  # No more tasks
                rospy.loginfo("Square pattern completed!")
                self.is_running = False
                return
                
            self.current_task = self.tasks.pop(0)
            self.current_task['start'] = self.right_ticks
            rospy.loginfo(f"Starting {self.current_task['action']}")
        
        # Execute current task
        task_complete = False
        if self.current_task['action'] == 'move':
            task_complete = self.move_ticks()
        elif self.current_task['action'] == 'turn_ticks':
            # Initialize on first run
            if 'start_left' not in self.current_task or 'start_right' not in self.current_task:
                self.current_task['start_left'] = self.left_ticks
                self.current_task['start_right'] = self.right_ticks
            task_complete = self.turn_ticks()

        # Handle task completion
        if task_complete:
            self.current_task = None
            self.stop_robot()
            
            if self.tasks:  # More tasks remaining
                self.break_start = rospy.Time.now()


    def move_ticks(self):
        ticks = self.current_task['ticks']

        # Initialize on first run
        if 'start_right' not in self.current_task or 'start_left' not in self.current_task:
            self.current_task['start_right'] = self.right_ticks
            self.current_task['start_left'] = self.left_ticks

            # Initial motor nudge
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = max(0.2, abs(self.linear_speed)) * (1 if ticks >= 0 else -1)
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            return False

        # Calculate how far we've moved
        delta_right = self.right_ticks - self.current_task['start_right']
        delta_left = self.left_ticks - self.current_task['start_left']
        average_ticks = (delta_right + delta_left) / 2.0

        # Check for completion
        if abs(average_ticks) >= abs(ticks):
            rospy.loginfo("Tick-based move complete.")
            return True

        # Continue moving
        direction = 1 if (ticks - average_ticks) > 0 else -1
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = abs(self.linear_speed) * direction
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        return False
    
    def turn_ticks(self):
        if not self.current_task or self.current_task['action'] != 'turn_ticks':
            return False

        ticks = self.current_task['ticks']
        start_left = self.current_task['start_left']
        start_right = self.current_task['start_right']
        angular_speed = self.current_task['angular_speed']

        # Calculate how far we’ve turned
        delta_right = self.right_ticks - start_right
        delta_left = self.left_ticks - start_left
        diff_ticks = (delta_right - delta_left) / 2.0

        if abs(diff_ticks) >= abs(ticks):
            rospy.loginfo("Turn_ticks completed.")
            return True

        # Continue turning
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = angular_speed
        self.pub.publish(self.cmd_msg)
        return False
    
    def start_turn_degrees(self, degrees, angular_speed):

        tick_sign = 1 if degrees >= 0 else -1
        ticks = int(abs(degrees) / 90.0 * self.ticks_per_90_deg) * tick_sign
        self.tasks.append({
            'action': 'turn_ticks',
            'ticks': ticks,
            'angular_speed': angular_speed
        })

    def run(self):
        rospy.spin()  # keeps node from exiting until node has shutdown

if __name__ == '__main__':
    try:
        closed_loop_square = ClosedLoopSquare()
        rospy.sleep(2.0)  # Give system time to initialize
        closed_loop_square.is_running = True  # Start sequence
        closed_loop_square.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in main: {str(e)}")
