import cv2
import numpy as np

def perspective_to_birdseye(image, homography_matrix):
    # Get the height and width of the input image
    height, width = image.shape[:2]

    # Create a bird's eye view image with the same width as the input image
    birdseye = np.zeros((height, width), dtype=np.uint8)

    # Perform the perspective transformation using the homography matrix
    birdseye = cv2.warpPerspective(image, homography_matrix, (width, height))

    return birdseye

# Load the input image
# image = cv2.imread('data/000036.png') 
image = cv2.imread('ipm/cityscape_sample.png')
image_vis = image.copy()
print(image.shape) # 1024, 2048, 3

# Define the source points (perspective view) and destination points (bird's eye view)
src_points = np.float32([[950, 500], [700, 800], [1180, 500], [1600, 800]])
dst_points = np.float32([[0,0], [0, 640], [640, 0], [640, 640]])

for src_point in src_points:
    cv2.circle(image_vis, [int(src_point[0]), int(src_point[1])], 4, (255, 0, 0), 2)
cv2.imshow('Selected points', image_vis)
cv2.waitKey(0)

# Calculate the homography matrix
homography_matrix = cv2.getPerspectiveTransform(src_points, dst_points)

# Perform the perspective to bird's eye view transformation
birdseye_view = perspective_to_birdseye(image, homography_matrix)

# Display the bird's eye view image
cv2.imshow('Birdseye View', birdseye_view)
cv2.waitKey(0)
cv2.destroyAllWindows()