#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()  # Corrected indentation here
        self.image_sub = rospy.Subscriber(
            '/akandb/camera_node/image/compressed',  # <-- Corrected topic name
            CompressedImage,
            self.image_callback,
            queue_size=1
        )

        rospy.init_node("my_lane_detector")

    def image_callback(self, msg):
        rospy.loginfo("image_callback")

        # Convert ROS compressed image to OpenCV BGR image
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Crop the image (adjust crop as needed to isolate road area)
        cropped = img[300:480, :]  # assuming 480p image (480x640)

        # Convert to HSV
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        # White color mask
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        white_result = cv2.bitwise_and(cropped, cropped, mask=white_mask)

        # Yellow color mask
        lower_yellow = np.array([15, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        yellow_result = cv2.bitwise_and(cropped, cropped, mask=yellow_mask)

        # Combine both masks for edge detection
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)

        # Edge detection
        edges = cv2.Canny(combined_mask, 50, 150)

        # Hough Line Transform on white and yellow masks
        lines_white = cv2.HoughLinesP(white_mask, 1, np.pi/180, 50, minLineLength=40, maxLineGap=20)
        lines_yellow = cv2.HoughLinesP(yellow_mask, 1, np.pi/180, 50, minLineLength=40, maxLineGap=20)

        # Draw lines on a copy of cropped image
        output = np.copy(cropped)
        self.draw_lines(output, lines_white, color=(255, 255, 255))   # White lines
        self.draw_lines(output, lines_yellow, color=(0, 255, 255))    # Yellow lines

        # Display output
        cv2.imshow('Lane Detection', output)
        cv2.waitKey(1)

    def draw_lines(self, image, lines, color=(255, 0, 0)):
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(image, (x1, y1), (x2, y2), color, 3)
                cv2.circle(image, (x1, y1), 3, (0, 255, 0), -1)
                cv2.circle(image, (x2, y2), 3, (0, 0, 255), -1)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        detector = Lane_Detector()
        detector.run()
    except rospy.ROSInterruptException:
        pass

