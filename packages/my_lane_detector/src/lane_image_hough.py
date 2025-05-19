import cv2 as cv
import matplotlib.pyplot as plt

# Load your saved image
img = cv.imread('/data/lane_image.png')
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# Apply Canny edge detection
edges = cv.Canny(gray, 50, 150)

# Parameters to vary
min_lengths = [20, 50, 100]  # Change this list to experiment

plt.figure(figsize=(15, 5))

for i, min_len in enumerate(min_lengths):
    # Apply Hough Line Transform
    lines = cv.HoughLinesP(edges, 1, cv.cv2.PI / 180, threshold=50,
                           minLineLength=min_len, maxLineGap=10)

    # Draw lines on a copy of original image
    line_img = img.copy()
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv.line(line_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # Display the result
    plt.subplot(1, 3, i + 1)
    plt.imshow(cv.cvtColor(line_img, cv.COLOR_BGR2RGB))
    plt.title(f"minLineLength={min_len}")
    plt.axis('off')

plt.suptitle("Hough Transform - Varying minLineLength")
plt.tight_layout()
plt.show()
