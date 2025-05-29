#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
from duckietown_msgs.msg import AprilTagDetectionArray

class Autopilot:
    def __init__(self):
        
        #Initialize ROS node
        rospy.init_node('autopilot_node', anonymous=True)

        self.robot_state = "LANE_FOLLOWING"
        
        # Stop sign control variables
        self.stop_sign_detected = False
        self.stop_sign_processed = False
        self.last_stop_time = None

        # When shutdown signal is received, we run clean_shutdown function
        rospy.on_shutdown(self.clean_shutdown)
        
        ###### Init Pub/Subs. REMEMBER TO REPLACE "akandb" WITH YOUR ROBOT'S NAME #####
        self.cmd_vel_pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        self.state_pub = rospy.Publisher('/birdie/fsm_node/mode', FSMState, queue_size=1)
        rospy.Subscriber('/birdie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        ################################################################

        rospy.spin() # Spin forever but listen to message callbacks

    # Apriltag Detection Callback
    def tag_callback(self, msg):
        if self.robot_state != "LANE_FOLLOWING":
            return
        
        self.move_robot(msg.detections)
 
    # Stop Robot before node has shut down. This ensures the robot keep moving with the latest velocity command
    def clean_shutdown(self):
        rospy.loginfo("System shutting down. Stopping robot...")
        self.stop_robot()

    # Sends zero velocity to stop the robot
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

    def move_robot(self, detections):

        #### YOUR CODE GOES HERE ####

        if len(detections) == 0:
            # If no detections and we were previously processing a stop sign, reset the flag
            if self.stop_sign_detected and not self.stop_sign_processed:
                self.reset_stop_sign_state()
            return

        # Check if any detection is a stop sign (tag ID 31)
        stop_sign_detection = None
        for detection in detections:
            if detection.tag_id == 31:
                stop_sign_detection = detection
                break
        
        if stop_sign_detection is not None:
            self.handle_stop_sign(stop_sign_detection)
        else:
            # If no stop sign detected, reset stop sign state
            if self.stop_sign_detected and not self.stop_sign_processed:
                self.reset_stop_sign_state()

        #############################
    
    def handle_stop_sign(self, detection):
        """Handle stop sign detection and stopping behavior"""
        
        # If we've already processed this stop sign recently, ignore it
        if self.stop_sign_processed:
            current_time = rospy.Time.now()
            if self.last_stop_time and (current_time - self.last_stop_time).to_sec() < 5.0:
                # Ignore stop signs for 5 seconds after processing one
                return
            else:
                # Reset if enough time has passed
                self.stop_sign_processed = False
        
        # Mark that we've detected a stop sign
        if not self.stop_sign_detected:
            self.stop_sign_detected = True
            rospy.loginfo("Stop sign detected! Initiating stop sequence...")
            
            # Switch to manual control to override lane following
            self.set_state("NORMAL_JOYSTICK_CONTROL")
            
            # Stop the robot
            self.stop_robot()
            rospy.loginfo("Robot stopped for 3 seconds...")
            
            # Wait for 3 seconds
            rospy.sleep(3.0)
            
            # Mark this stop sign as processed
            self.stop_sign_processed = True
            self.last_stop_time = rospy.Time.now()
            
            rospy.loginfo("3 seconds completed. Resuming lane following...")
            
            # Return to lane following
            self.set_state("LANE_FOLLOWING")
            
            # Reset detection flag after a brief delay
            rospy.Timer(rospy.Duration(1.0), self.reset_stop_sign_callback, oneshot=True)
    
    def reset_stop_sign_callback(self, event):
        """Timer callback to reset stop sign detection state"""
        self.reset_stop_sign_state()
    
    def reset_stop_sign_state(self):
        """Reset stop sign detection state"""
        self.stop_sign_detected = False
        rospy.loginfo("Stop sign state reset. Ready to detect new stop signs.")

if __name__ == '__main__':
    try:
        autopilot_instance = Autopilot()
    except rospy.ROSInterruptException:
        pass
