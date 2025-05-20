import cv2 as cv
import numpy as np

def detect_lanes(image_path):
    frame = cv.imread(image_path)
    height, width, _ = frame.shape

    # Crop just a little from the top (1/6)
    cropped = frame[int(height / 6):, :]

    hsv = cv.cvtColor(cropped, cv.COLOR_BGR2HSV)

    # Masks
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    white_mask = cv.inRange(hsv, lower_white, upper_white)
    white_filtered = cv.bitwise_and(cropped, cropped, mask=white_mask)

    lower_yellow = np.array([15, 30, 20])
    upper_yellow = np.array([35, 255, 255])
    yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)
    yellow_filtered = cv.bitwise_and(cropped, cropped, mask=yellow_mask)

    # Grayscale and edges
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

    return output

# Detect lanes on both images
output_dark = detect_lanes('/data/lane_dark.png')
output_bright = detect_lanes('/data/lane_bright.png')

# Optionally resize to make each image smaller (to simulate "zoomed out" view)
scale = 0.75  # or 0.5 for even more zoomed-out effect
output_dark_resized = cv.resize(output_dark, None, fx=scale, fy=scale, interpolation=cv.INTER_AREA)
output_bright_resized = cv.resize(output_bright, None, fx=scale, fy=scale, interpolation=cv.INTER_AREA)

# Combine side-by-side
combined = cv.hconcat([output_dark_resized, output_bright_resized])

# Show and save
cv.imshow("Zoomed-Out Lane Detection", combined)
cv.imwrite("zoomed_out_comparison.png", combined)
cv.waitKey(0)
cv.destroyAllWindows()

