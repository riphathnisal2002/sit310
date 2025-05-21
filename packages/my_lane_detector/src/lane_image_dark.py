import cv2 as cv
import numpy as np

def detect_lanes(image_path):
    frame = cv.imread(image_path)
    height, width, _ = frame.shape
    # Removed the cropping line and kept the full image
    # cropped = frame[int(height / 5):, :]  # No cropping now

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
    output = frame.copy()  # No cropping here
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
output_dark = detect_lanes('/data/lane_dark.png')
output_bright = detect_lanes('/data/lane_bright.png')

# Make output bigger (lengthwise)
scale = 1.5  # Try 2.0 if you want it even wider
output_dark_resized = cv.resize(output_dark, None, fx=scale, fy=scale, interpolation=cv.INTER_LINEAR)
output_bright_resized = cv.resize(output_bright, None, fx=scale, fy=scale, interpolation=cv.INTER_LINEAR)

# Combine side-by-side
combined = cv.hconcat([output_dark_resized, output_bright_resized])

# Show and save
cv.imshow("Lane Detection - Dark vs Bright (Wider View)", combined)
cv.imwrite("lane_comparison_wide.png", combined)
cv.waitKey(0)
cv.destroyAllWindows()

