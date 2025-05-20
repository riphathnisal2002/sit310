import cv2 as cv
import numpy as np

def detect_lanes(image_path):
    frame = cv.imread(image_path)

    height, width, _ = frame.shape
    cropped = frame[int(height / 3):, :]

    hsv = cv.cvtColor(cropped, cv.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    white_mask = cv.inRange(hsv, lower_white, upper_white)
    white_filtered = cv.bitwise_and(cropped, cropped, mask=white_mask)

    lower_yellow = np.array([15, 30, 20])
    upper_yellow = np.array([35, 255, 255])
    yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)
    yellow_filtered = cv.bitwise_and(cropped, cropped, mask=yellow_mask)

    white_gray = cv.cvtColor(white_filtered, cv.COLOR_BGR2GRAY)
    yellow_gray = cv.cvtColor(yellow_filtered, cv.COLOR_BGR2GRAY)

    white_edges = cv.Canny(white_gray, 50, 150)
    yellow_edges = cv.Canny(yellow_gray, 50, 150)

    white_lines = cv.HoughLinesP(white_edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=10)
    yellow_lines = cv.HoughLinesP(yellow_edges, 1, np.pi / 360, 10, minLineLength=10, maxLineGap=10)

    output = cropped.copy()
    if white_lines is not None:
        for l in white_lines:
            x1, y1, x2, y2 = l[0]
            cv.line(output, (x1, y1), (x2, y2), (0, 0, 255), 2)
    if yellow_lines is not None:
        for l in yellow_lines:
            x1, y1, x2, y2 = l[0]
            cv.line(output, (x1, y1), (x2, y2), (0, 255, 255), 2)

    cv.imshow(f"Lane Detection - {image_path}", output)
    cv.imwrite(f"{image_path.replace('.png', '_lanes.png')}", output)
    cv.waitKey(0)
    cv.destroyAllWindows()

# Run detection on both lighting conditions
detect_lanes('/data/lane_dark.png')
detect_lanes('/data/lane_bright.png')
