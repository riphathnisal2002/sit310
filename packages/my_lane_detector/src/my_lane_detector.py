#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class LaneDetector:
    def __init__(self):
        rospy.init_node('lane_detector_node')
        self.bridge = CvBridge()
        rospy.Subscriber("/akandb/camera_node/image/compressed", CompressedImage, self.callback)
        rospy.loginfo("Lane Detector Node Started.")
        rospy.spin()

    def callback(self, msg):
        # Decode the compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv.imdecode(np_arr, cv.IMREAD_COLOR)

        # Crop the image to focus on a larger road area (for example, upper half)
        height, width, _ = frame.shape
        cropped = frame[int(height / 3):, :]  # Adjust the crop area to focus on a larger part of the road

        # Convert to HSV
        hsv = cv.cvtColor(cropped, cv.COLOR_BGR2HSV)

        # White filter
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        white_mask = cv.inRange(hsv, lower_white, upper_white)
        white_filtered = cv.bitwise_and(cropped, cropped, mask=white_mask)

        # Yellow filter
        lower_yellow = np.array([15, 30, 20])
        upper_yellow = np.array([35, 255, 255])
        yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)
        yellow_filtered = cv.bitwise_and(cropped, cropped, mask=yellow_mask)

        # Convert both filtered images to grayscale for edge detection
        white_gray = cv.cvtColor(white_filtered, cv.COLOR_BGR2GRAY)
        yellow_gray = cv.cvtColor(yellow_filtered, cv.COLOR_BGR2GRAY)

        # Apply Canny edge detector
        white_edges = cv.Canny(white_gray, 50, 150)
        yellow_edges = cv.Canny(yellow_gray, 50, 150)

        # Detect lines using Probabilistic Hough Transform
        white_lines = cv.HoughLinesP(white_edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=10)
        yellow_lines = cv.HoughLinesP(yellow_edges, 1, np.pi / 360, 10, minLineLength=10, maxLineGap=10)

        # Copy for drawing
        output_lines = cropped.copy()

        if white_lines is not None:
            for l in white_lines:
                x1, y1, x2, y2 = l[0]
                cv.line(output_lines, (x1, y1), (x2, y2), (0, 0, 255), 2)  # Change white lines to red (BGR format)

        if yellow_lines is not None:
            for l in yellow_lines:
                x1, y1, x2, y2 = l[0]
                cv.line(output_lines, (x1, y1), (x2, y2), (0, 255, 255), 2)

        # Display all three windows
        cv.imshow("White Filtered Image", white_filtered)
        cv.imshow("Yellow Filtered Image", yellow_filtered)
        cv.imshow("Lane Lines on Road", output_lines)
        cv.waitKey(1)

if __name__ == '__main__':
    try:
        LaneDetector()
    except rospy.ROSInterruptException:
        pass

