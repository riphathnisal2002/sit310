#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)
        self.cmd_vel_pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/birdie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        
        # Control parameters
        self.rotation_speed = 3.0    # Fixed rotation speed (positive = counter-clockwise)
        self.rotation_direction = 1  # 1 = counter-clockwise, -1 = clockwise
        self.rotation_margin = 0.1   # How far tag can move before robot responds
        
        # State variables
        self.last_tag_x = None
        self.tracking_active = False
        
        rospy.spin()
        
    def tag_callback(self, msg):
        self.move_robot(msg.detections)
        
    def clean_shutdown(self):
        rospy.loginfo("Shutting down. Stopping robot...")
        self.stop_robot()
        
    def stop_robot(self):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0
        cmd_msg.omega = 0.0
        self.cmd_vel_pub.publish(cmd_msg)
        
    def move_robot(self, detections):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.v = 0.0  # Always keep robot stationary (no forward/backward movement)
        
        # --- No tag detected ---
        if len(detections) == 0:
            rospy.loginfo("No tag detected. Spinning in search direction...")
            cmd_msg.omega = self.rotation_speed * self.rotation_direction
            self.tracking_active = False
            self.last_tag_x = None
            self.cmd_vel_pub.publish(cmd_msg)
            return
            
        # --- Tag detected ---
        closest_tag = min(detections, key=lambda tag: abs(tag.transform.translation.x))
        x = closest_tag.transform.translation.x
        
        rospy.loginfo("Tracking tag. x: %.3f", x)
        
        # Initialize last_tag_x if this is the first detection
        if self.last_tag_x is None:
            self.last_tag_x = x
        
        # Determine if we need to rotate based on tag movement
        if not self.tracking_active:
            # Start rotation if tag is not centered
            if abs(x) > self.rotation_margin:
                self.tracking_active = True
        else:
            # Check if tag has moved significantly from last position
            tag_moved = abs(x - self.last_tag_x) > 0.01
            
            # Check if tag is now centered enough to stop
            if abs(x) < self.rotation_margin:
                self.tracking_active = False
        
        # Save current position for next comparison
        self.last_tag_x = x
        
        # Apply rotation logic
        if self.tracking_active:
            # Rotate in the predefined direction at fixed speed
            cmd_msg.omega = self.rotation_speed * self.rotation_direction
            rospy.loginfo("Tag tracking active - rotating")
        else:
            # Tag is centered enough - stop rotating
            cmd_msg.omega = 0.0
            rospy.loginfo("Tag centered - stopped")
            
        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
