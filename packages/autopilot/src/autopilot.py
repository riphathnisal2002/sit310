#!/usr/bin/env python3
import rospy
import numpy as np
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class EnhancedLaneFollower:
    def __init__(self):
        rospy.init_node('enhanced_lane_follower', anonymous=True)
        
        # Publishers and Subscribers
        self.cmd_pub = rospy.Publisher('/birdie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/birdie/apriltag_detector_node/detections', AprilTagDetectionArray, self.tag_callback)
        rospy.Subscriber('/birdie/camera_node/image/compressed', CompressedImage, self.image_callback)
        
        # CV Bridge for image processing
        self.bridge = CvBridge()
        
        # Control parameters
        self.base_speed = 0.3  # Base forward speed
        self.max_angular_speed = 2.0  # Maximum turning speed
        
        # PID controller parameters for lane following
        self.kp = 0.8  # Proportional gain
        self.ki = 0.05  # Integral gain  
        self.kd = 0.1  # Derivative gain
        
        # PID state variables
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.last_time = rospy.Time.now()
        
        # Tag detection state
        self.last_tag_time = rospy.Time(0)
        self.tag_cooldown = rospy.Duration(5.0)
        self.is_stopped = False
        
        # Lane detection parameters
        self.image_width = 640
        self.image_height = 480
        self.roi_height = 200  # Region of interest height from bottom
        
        rospy.on_shutdown(self.stop_robot)
        
        # Main control loop
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)  # 20Hz
        rospy.spin()
    
    def image_callback(self, msg):
        """Process camera image for lane detection"""
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Process image for lane detection
            self.lane_error = self.detect_lanes(cv_image)
            
        except Exception as e:
            rospy.logwarn(f"Image processing error: {e}")
            self.lane_error = 0.0
    
    def detect_lanes(self, image):
        """Detect lane lines and return steering error"""
        # Convert to HSV for better color filtering
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define yellow and white color ranges for lane detection
        # Yellow lane markers
        yellow_lower = np.array([20, 50, 50])
        yellow_upper = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        
        # White lane markers
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        
        # Combine masks
        lane_mask = cv2.bitwise_or(yellow_mask, white_mask)
        
        # Focus on region of interest (bottom portion of image)
        roi_mask = np.zeros_like(lane_mask)
        roi_mask[-self.roi_height:, :] = 255
        lane_mask = cv2.bitwise_and(lane_mask, roi_mask)
        
        # Find contours
        contours, _ = cv2.findContours(lane_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return 0.0  # No lanes detected, go straight
        
        # Find the largest contour (assumed to be the lane)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Calculate moments to find centroid
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            lane_center_x = int(M["m10"] / M["m00"])
            image_center_x = self.image_width // 2
            
            # Calculate error (positive = turn right, negative = turn left)
            error = (lane_center_x - image_center_x) / (self.image_width / 2.0)
            return error
        
        return 0.0
    
    def pid_controller(self, error):
        """PID controller for smooth steering"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        if dt <= 0:
            return 0.0
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral_error += error * dt
        integral = self.ki * self.integral_error
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Calculate output
        output = proportional + integral + derivative
        
        # Update for next iteration
        self.previous_error = error
        self.last_time = current_time
        
        # Clamp output to maximum angular speed
        return np.clip(output, -self.max_angular_speed, self.max_angular_speed)
    
    def tag_callback(self, msg):
        """Handle AprilTag detections"""
        now = rospy.Time.now()
        if len(msg.detections) > 0 and (now - self.last_tag_time > self.tag_cooldown):
            rospy.loginfo("Tag detected! Initiating stop sequence.")
            self.last_tag_time = now
            self.is_stopped = True
            
            # Stop the robot
            self.stop_robot()
            
            # Sleep for 3 seconds
            rospy.sleep(3.0)
            
            # Reset PID controller state
            self.integral_error = 0.0
            self.previous_error = 0.0
            
            self.is_stopped = False
            rospy.loginfo("Stop sequence complete, resuming lane following.")
    
    def control_loop(self, event):
        """Main control loop for lane following"""
        if self.is_stopped:
            return
        
        # Get lane following command
        if hasattr(self, 'lane_error'):
            # Use PID controller for smooth steering
            angular_velocity = -self.pid_controller(self.lane_error)  # Negative for correct turning direction
            
            # Reduce speed when turning sharply
            speed_factor = 1.0 - 0.3 * abs(angular_velocity) / self.max_angular_speed
            linear_velocity = self.base_speed * speed_factor
            
        else:
            # No lane detected yet, go straight slowly
            linear_velocity = self.base_speed * 0.5
            angular_velocity = 0.0
        
        # Publish command
        self.publish_cmd(linear_velocity, angular_velocity)
    
    def publish_cmd(self, v, omega):
        """Publish movement command"""
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = v
        cmd.omega = omega
        self.cmd_pub.publish(cmd)
    
    def stop_robot(self):
        """Stop the robot completely"""
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = 0.0
        cmd.omega = 0.0
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        EnhancedLaneFollower()
    except rospy.ROSInterruptException:
        rospy.loginfo("Lane follower node interrupted.")
        pass
