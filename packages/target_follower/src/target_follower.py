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
        if self.shutting_down:
            return  # Avoid publishing if node is shutting down

        cmd_msg = Twist2DStamped()
        cmd_msg.header.stamp = rospy.Time.now()

        if len(detections) == 0:
            rospy.loginfo("No tag detected. Staying still.")
            cmd_msg.v = 0.0
            cmd_msg.omega = 0.0
            self.cmd_vel_pub.publish(cmd_msg)
            return

        transform = detections[0].transform.translation
        x = transform.x  # Horizontal offset (left/right)
        y = transform.y
        z = transform.z  # Forward distance to tag

        rospy.loginfo("Tracking tag. x: %.3f, y: %.3f, z: %.3f", x, y, z)

        # --- ANGULAR CONTROL ---
        angular_dead_zone = 0.05  # Ignore small deviations

        if abs(x) > angular_dead_zone:
            omega = self.Kp_angular * x

            if abs(omega) < self.omega_min:
                omega = self.omega_min * (1 if omega > 0 else -1)

            omega = max(-self.omega_max, min(self.omega_max, omega))
        else:
            omega = 0.0  # Within dead zone, no need to turn

        # --- LINEAR CONTROL ---
        distance_error = z - self.desired_distance
        v = self.Kp_linear * distance_error

        if abs(distance_error) < 0.05:
            v = 0.0  # Close enough to stop moving forward/backward

        v = max(-self.v_max, min(self.v_max, v))

        cmd_msg.v = v
        cmd_msg.omega = omega
        self.cmd_vel_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        target_follower = Target_Follower()
    except rospy.ROSInterruptException:
        pass
