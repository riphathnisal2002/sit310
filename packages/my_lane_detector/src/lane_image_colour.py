import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

# Load image
img = cv.imread('/data/lane_image.jpg')

# Resize if needed
# img = cv.resize(img, (640, 480))

# ----------- HSV Process -----------
hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
lower_yellow_hsv = np.array([15, 30, 20])
upper_yellow_hsv = np.array([35, 255, 255])
mask_hsv = cv.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
result_hsv = cv.bitwise_and(img, img, mask=mask_hsv)

# ----------- RGB Process -----------
rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
lower_yellow_rgb = np.array([180, 180, 0])
upper_yellow_rgb = np.array([255, 255, 150])
mask_rgb = cv.inRange(rgb, lower_yellow_rgb, upper_yellow_rgb)
result_rgb = cv.bitwise_and(img, img, mask=mask_rgb)

# ----------- Show Results -----------
plt.figure(figsize=(12, 6))

plt.subplot(1, 2, 1)
plt.imshow(cv.cvtColor(result_hsv, cv.COLOR_BGR2RGB))
plt.title("Yellow Detection (HSV)")
plt.axis('off')

plt.subplot(1, 2, 2)
plt.imshow(cv.cvtColor(result_rgb, cv.COLOR_BGR2RGB))
plt.title("Yellow Detection (RGB)")
plt.axis('off')

plt.tight_layout()
plt.show()
