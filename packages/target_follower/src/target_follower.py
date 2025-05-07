#!/usr/bin/env python3
import rospy
import time
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class Target_Follower:
    def __init__(self):
        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)
        self.cmd_vel_pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/birdie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback, queue_size=1)
        
        # Control parameters
        self.Kp_omega = 4.5     # Proportional gain for rotation
        self.omega_min = 2.0    # Minimum omega to overcome friction
        self.omega_max = 6.0    # Maximum rotation speed
        
        # Debug flags
        self.test_directions = True  # Enable direction testing
        self.last_test_time = rospy.Time.now()
        self.test_direction = 1      # Start with counter-clockwise
        
        rospy.spin()
        
    def tag_callback(self, msg):
        if self.test_directions:
            self.test_rotation_directions()
        else:
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
    
    def test_rotation_directions(self):
        """Test both rotation directions to verify robot can turn both ways"""
        now = rospy.Time.now()
        duration = 3.0  # Test each direction for 3 seconds
        
        # Switch direction every 'duration' seconds
        if (now - self.last_test_time).to_sec() > duration:
            self.test_direction *= -1  # Flip direction
            self.last_test_time = now
            rospy.loginfo("===== DIRECTION TEST: Switching to %s =====" % 
                         ("CLOCKWISE" if self.test_direction < 0 else "COUNTER-CLOCKWISE"))
        
        # Apply test rotation
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = now
        cmd_msg.v = 0.0
        cmd_msg.omega = self.test_direction * 3.0  # Use a moderate speed
        
        rospy.loginfo("Direction test: omega = %.1f", cmd_msg.omega)
        self.cmd_vel_pub.publish(cmd_msg)
    
    def move_robot(self, detections):
        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()
        
        # --- Seek Mode: No tag detected ---
        if len(detections) == 0:
            rospy.loginfo("No tag detected. Seeking...")
            cmd_msg.v = 0.0
            cmd_msg.omega = 5.0  # Spin to search for tag
            self.cmd_vel_pub.publish(cmd_msg)
            return
            
        # --- Tag detected: Pick the one closest to center (smallest |x|) ---
        closest_tag = min(detections, key=lambda tag: abs(tag.transform.translation.x))
        x = closest_tag.transform.translation.x
        y = closest_tag.transform.translation.y
        z = closest_tag.transform.translation.z
        
        # Print raw tag position data for debugging
        rospy.loginfo("Tracking tag. x: %.3f, y: %.3f, z: %.3f", x, y, z)
        try:
            rospy.loginfo("Tag ID: %d", closest_tag.id)
        except:
            rospy.loginfo("Tag ID not available")
        
        # Rotation control (to center the tag)
        # IMPORTANT: Explicitly check direction and apply appropriate sign
        if x > 0:
            # Tag is to the right, need to rotate clockwise (negative omega)
            direction = -1
            rospy.loginfo("Tag is to the RIGHT, rotating CLOCKWISE")
        else:
            # Tag is to the left, need to rotate counter-clockwise (positive omega)
            direction = 1
            rospy.loginfo("Tag is to the LEFT, rotating COUNTER-CLOCKWISE")
        
        # Compute rotation with explicit direction control
        omega = direction * self.Kp_omega * abs(x)
        
        # Apply minimum threshold to overcome static friction
        if abs(omega) < self.omega_min and abs(x) > 0.01:
            omega = self.omega_min * direction
            rospy.loginfo("Applied minimum omega threshold: %.3f", omega)
            
        # Clamp omega to safe limits
        omega = max(-self.omega_max, min(self.omega_max, omega))
        
        # Force rotation direction based on tag position for small errors
        if abs(x) > 0.005:  # Only if tag is not perfectly centered
            # Ensure at least some movement in the right direction
            if (x > 0 and omega >= 0) or (x < 0 and omega <= 0):
                # Direction is wrong, force minimum rotation in correct direction
                omega = self.omega_min * direction
                rospy.loginfo("Forcing correct rotation direction: %.3f", omega)
        
        # Override omega for debugging (uncomment to test specific directions)
        # if x > 0:
        #    omega = -3.0  # Force clockwise rotation
        # else:
        #    omega = 3.0   # Force counter-clockwise rotation
        
        rospy.loginfo("Control: v=0.0, omega=%.3f", omega)
        
        # Always keep v=0 to ensure robot stays in place and only rotates
        cmd_msg.v = 0.0
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
