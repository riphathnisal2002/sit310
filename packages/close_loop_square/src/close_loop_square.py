#!/usr/bin/env python3

import rospy 
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped, FSMState

class ClosedLoopSquare:
    def __init__(self):
        # Initialize message
        self.cmd_msg = Twist2DStamped()

        # Initialize ROS node
        rospy.init_node('closed_loop_square_node', anonymous=True)
        rospy.loginfo("Node initialized")
        
        # Initialize publishers/subscribers
        self.pub = rospy.Publisher('/jeff/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/jeff/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback)
        rospy.Subscriber('/jeff/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        
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

        # Task queue to make a square
        for i in range(4):
            self.tasks.append({
                'action': 'move',
                'ticks': self.ticks_per_meter
            })
            self.tasks.append({
                'action': 'turn',
                'ticks': self.ticks_per_90_deg
            })

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
            for i in range(4):
                self.tasks.append({
                    'action': 'move',
                    'ticks': self.ticks_per_meter
                })
                self.tasks.append({
                    'action': 'turn',
                    'ticks': self.ticks_per_90_deg
                })
 
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def encoder_callback(self, msg):
        self.right_ticks = msg.data

    def move_distance(self, ticks, start):
        # Calculate progress and direction
        current_ticks = self.right_ticks - start
        remaining_ticks = ticks - current_ticks
        
        # Check if we've reached target (in either direction)
        if abs(current_ticks) >= abs(ticks):
            rospy.loginfo("Move complete")
            self.stop_robot()
            return True
        
        # Move in appropriate direction
        direction = 1 if remaining_ticks > 0 else -1
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = self.linear_speed * direction
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        return False
    
    def turn_90_deg(self, ticks, start):
        # Calculate progress and direction
        current_ticks = self.right_ticks - start
        remaining_ticks = ticks - current_ticks
        
        # Check if we've reached target angle (in either direction)
        if abs(current_ticks) >= abs(ticks):
            rospy.loginfo("Turn complete")
            self.stop_robot()
            return True
        
        # Turn in appropriate direction
        direction = 1 if remaining_ticks > 0 else -1
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = self.angular_speed * direction
        self.pub.publish(self.cmd_msg)
        return False
    
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
            task_complete = self.move_distance(self.current_task['ticks'], self.current_task['start'])
        elif self.current_task['action'] == 'turn':
            task_complete = self.turn_90_deg(self.current_task['ticks'], self.current_task['start'])
        
        # Handle task completion
        if task_complete:
            self.current_task = None
            self.stop_robot()
            
            if self.tasks:  # More tasks remaining
                self.break_start = rospy.Time.now()

    def move_custom_distance(self, distance_meters, speed):
        """
        Moves the robot a given distance at a specified speed.
        Handles both forward (positive) and backward (negative) motion.
        """
        rospy.loginfo(f"Starting custom move: {distance_meters} meters at {speed} m/s")

        # Calculate how many encoder ticks are needed for the distance
        target_ticks = int(distance_meters * self.ticks_per_meter)
        start_ticks = self.right_ticks
        rate = rospy.Rate(100)  # 100 Hz loop

        while not rospy.is_shutdown():
            current_ticks = self.right_ticks - start_ticks
            if abs(current_ticks) >= abs(target_ticks):
                rospy.loginfo("Custom move complete.")
                break

            direction = 1 if target_ticks - current_ticks > 0 else -1
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = abs(speed) * direction
            self.cmd_msg.omega = 0.0
            self.pub.publish(self.cmd_msg)
            rate.sleep()

        self.stop_robot()


    def run(self):
        rospy.spin()  # keeps node from exiting until node has shutdown

if __name__ == '__main__':
    try:
        closed_loop_square = ClosedLoopSquare()
        rospy.sleep(2.0)  # Allow ROS to initialize

        # Forward and back at 0.3 m/s
        closed_loop_square.move_custom_distance(1.0, 0.3)
        rospy.sleep(1.0)
        closed_loop_square.move_custom_distance(-1.0, 0.3)

        rospy.sleep(2.0)

        # Forward and back at 0.6 m/s
        closed_loop_square.move_custom_distance(1.0, 0.6)
        rospy.sleep(1.0)
        closed_loop_square.move_custom_distance(-1.0, 0.6)

        # Keep node alive
        closed_loop_square.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in main: {str(e)}")
