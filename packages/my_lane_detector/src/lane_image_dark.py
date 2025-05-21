import cv2 as cv
import numpy as np

def detect_lanes(image_path):
    frame = cv.imread(image_path)
    
    if frame is None:
        raise FileNotFoundError(f"Could not read image from path: {image_path}")

    height, width, _ = frame.shape

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    # White mask
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    white_mask = cv.inRange(hsv, lower_white, upper_white)
    white_filtered = cv.bitwise_and(frame, frame, mask=white_mask)

    # Yellow mask
    lower_yellow = np.array([15, 30, 20])
    upper_yellow = np.array([35, 255, 255])
    yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)
    yellow_filtered = cv.bitwise_and(frame, frame, mask=yellow_mask)

    # Edge detection
    white_gray = cv.cvtColor(white_filtered, cv.COLOR_BGR2GRAY)
    yellow_gray = cv.cvtColor(yellow_filtered, cv.COLOR_BGR2GRAY)
    white_edges = cv.Canny(white_gray, 50, 150)
    yellow_edges = cv.Canny(yellow_gray, 50, 150)

    # Hough Line detection
    white_lines = cv.HoughLinesP(white_edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=10)
    yellow_lines = cv.HoughLinesP(yellow_edges, 1, np.pi / 360, 10, minLineLength=10, maxLineGap=10)

    # Draw lines
    output = frame.copy()
    if white_lines is not None:
        for l in white_lines:
            x1, y1, x2, y2 = l[0]
            cv.line(output, (x1, y1), (x2, y2), (0, 0, 255), 2)
    if yellow_lines is not None:
        for l in yellow_lines:
            x1, y1, x2, y2 = l[0]
            cv.line(output, (x1, y1), (x2, y2), (0, 255, 255), 2)

    return output

# Load and process both full images
output_dark = detect_lanes('/data/dark_lane.png')
output_bright = detect_lanes('/data/bright_lane.png')

# Resize images if needed (scale = 1 means original size)
scale = 1
output_dark_resized = cv.resize(output_dark, None, fx=scale, fy=scale, interpolation=cv.INTER_LINEAR)
output_bright_resized = cv.resize(output_bright, None, fx=scale, fy=scale, interpolation=cv.INTER_LINEAR)

# Show each image separately
cv.imshow("Lane Detection - Dark", output_dark_resized)
cv.imshow("Lane Detection - Bright", outpu

