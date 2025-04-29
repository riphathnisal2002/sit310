#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, FSMState, WheelEncoderStamped

class DriveSquareClosedLoop:
    def __init__(self):
        # Initialize global class variables
        self.cmd_msg = Twist2DStamped()
        self.pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        
        rospy.init_node('drive_square_closed_loop_node', anonymous=True)
        
        # Subscribers
        rospy.Subscriber('/birdie/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        rospy.Subscriber('/birdie/right_wheel_encoder_node/tick', WheelEncoderStamped, self.encoder_callback, queue_size=1)

        # Encoder tracking
        self.start_ticks = None
        self.current_ticks = None
        
        # Parameters (you MUST measure experimentally!)
        self.ticks_per_meter = 500  # Example value! Replace with your own measured value
        self.ticks_per_radian = 150  # Example value! Replace with your measured value
        
        # Motion control
        self.moving = False
        self.goal_tick_delta = 0
        self.motion_type = None  # 'straight' or 'rotate'

        self.side_count = 0  # Track which side of the square we are on
        
    def fsm_callback(self, msg):
        rospy.loginfo("FSM State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)  # small delay for system to stabilize
            self.side_count = 0
            self.move_straight(1.0)  # move forward 1 meter to start
        
    def encoder_callback(self, msg):
        self.current_ticks = msg.data
        
        if not self.moving:
            return
        
        if self.start_ticks is None:
            self.start_ticks = self.current_ticks
            return
        
        tick_delta = abs(self.current_ticks - self.start_ticks)
        
        if tick_delta >= abs(self.goal_tick_delta):
            rospy.loginfo("Goal reached!")
            self.stop_robot()

            # After each motion complete, decide next action
            if self.motion_type == "straight":
                rospy.sleep(0.5)  # short break
                self.rotate_in_place(1.57)  # rotate 90 degrees (1.57 rad)
            elif self.motion_type == "rotate":
                self.side_count += 1
                if self.side_count < 4:
                    rospy.sleep(0.5)  # short break
                    self.move_straight(1.0)  # next side
                else:
                    rospy.loginfo("Finished the square!")
    
    def move_straight(self, distance_meters):
        rospy.loginfo(f"Starting to move straight for {distance_meters} meters.")
        self.start_ticks = None
        self.goal_tick_delta = distance_meters * self.ticks_per_meter
        self.moving = True
        self.motion_type = "straight"
        
        # Send constant forward velocity
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.3  # linear velocity (m/s)
        self.cmd_msg.omega = 0.0  # no rotation
        self.pub.publish(self.cmd_msg)

    def rotate_in_place(self, angle_radians):
        rospy.loginfo(f"Starting to rotate in place for {angle_radians} radians.")
        self.start_ticks = None
        self.goal_tick_delta = angle_radians * self.ticks_per_radian
        self.moving = True
        self.motion_type = "rotate"
        
        # Send constant angular velocity
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0  # no forward movement
        self.cmd_msg.omega = 3.0  # rotation speed (rad/s)
        self.pub.publish(self.cmd_msg)

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        self.moving = False
        self.start_ticks = None  # reset for next movement
        rospy.loginfo("Robot stopped.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        closed_loop_driver = DriveSquareClosedLoop()
        closed_loop_driver.run()
    except rospy.ROSInterruptException:
        pass
