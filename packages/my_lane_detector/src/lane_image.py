import cv2 as cv
import matplotlib.pyplot as plt

# Load the image
img = cv.imread('/home/ubuntu/sit310/packages/my_lane_detector/image_lane.png')
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# Threshold pairs to test
thresholds = [(30, 100), (50, 150), (100, 200)]
titles = [f'Low: {t[0]}, High: {t[1]}' for t in thresholds]

# Create plots
plt.figure(figsize=(15, 5))
for i, (low, high) in enumerate(thresholds):
    edges = cv.Canny(gray, low, high)
    plt.subplot(1, 3, i + 1)
    plt.imshow(edges, cmap='gray')
    plt.title(titles[i])
    plt.axis('off')

plt.suptitle("Canny Edge Detection with Varying Thresholds")
plt.tight_layout()
plt.savefig('/home/ubuntu/Documents/canny_results.png')
plt.show()
