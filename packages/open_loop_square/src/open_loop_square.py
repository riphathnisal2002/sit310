#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
 
class Drive_Square:
    def __init__(self):
        #Initialize global class variables
        self.cmd_msg = Twist2DStamped()

        #Initialize ROS node
        rospy.init_node('drive_square_node', anonymous=True)
        
        #Initialize Pub/Subs
        self.pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/birdie/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)
        
    # robot only moves when lane following is selected on the duckiebot joystick app
    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":            
            rospy.sleep(1) # Wait for a sec for the node to be ready
            self.move_robot()
 
    # Sends zero velocities to stop the robot
    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
 
    # Spin forever but listen to message callbacks
    def run(self):
    	rospy.spin() # keeps node from exiting until node has shutdown

    # Robot drives in a square and then stops
    def move_robot(self):

       # Move in a square: 4 sides and 4 turns
        for i in range(4):
            # Move forward 1 meter
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 5.5  # linear velocity (m/s)
            self.cmd_msg.omega = 0.0  # no angular velocity
            self.pub.publish(self.cmd_msg)
            rospy.loginfo(f"Moving forward - Side {i+1}")
            rospy.sleep(2)  # Move forward for 2 seconds to cover 1 meter
            
            # Stop movement
            self.stop_robot()
            rospy.sleep(0.2)

            # Rotate 90 degrees
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0.3
            self.cmd_msg.omega = 8.3  # angular velocity (rad/s)
            self.pub.publish(self.cmd_msg)
            rospy.loginfo("Turning 90 degrees")
            rospy.sleep(1.57)  # Rotate 90 degrees (π/2 radians)
            
            self.stop_robot()
            rospy.sleep(0.5)

        # Stop after completing the square
        self.stop_robot()
        rospy.loginfo("Finished square path and stopped.")

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
