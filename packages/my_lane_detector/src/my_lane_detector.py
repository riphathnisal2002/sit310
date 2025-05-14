#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class LaneDetectorNode:
    def __init__(self):
        rospy.init_node("lane_detector_node")
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback)
        rospy.loginfo("Lane Detector Node Started.")
        rospy.spin()

    def callback(self, msg):
        # Convert image
        img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Crop (adjust as needed)
        cropped = img[300:480, :]

        # Convert to HSV
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        # White filter
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        white_filtered = cv2.bitwise_and(cropped, cropped, mask=white_mask)

        # Yellow filter
        lower_yellow = np.array([15, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        yellow_filtered = cv2.bitwise_and(cropped, cropped, mask=yellow_mask)

        # Canny Edge (on cropped image)
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # Hough Transform
        lines_white = cv2.HoughLinesP(white_mask, 1, np.pi / 180, 50, minLineLength=40, maxLineGap=20)
        lines_yellow = cv2.HoughLinesP(yellow_mask, 1, np.pi / 180, 50, minLineLength=40, maxLineGap=20)

        # Copy to draw
        output = cropped.copy()
        self.draw_lines(output, lines_white, (255, 255, 255))  # white lines in white
        self.draw_lines(output, lines_yellow, (0, 255, 255))  # yellow lines in yellow

        # Show OpenCV windows
        cv2.imshow("White Filtered Image", white_filtered)
        cv2.imshow("Yellow Filtered Image", yellow_filtered)
        cv2.imshow("Hough Lines Output", output)
        cv2.waitKey(1)

    def draw_lines(self, img, lines, color):
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), color, 3)

if __name__ == "__main__":
    try:
        LaneDetectorNode()
    except rospy.ROSInterruptException:
        pass
