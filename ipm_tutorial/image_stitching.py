import cv2
import numpy as np

# Load input images
image1 = cv2.imread("data/img1.png")
image2 = cv2.imread("data/img2.png")

gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

# Detect keypoints and extract descriptors
sift = cv2.SIFT_create()
keypoints1, descriptors1 = sift.detectAndCompute(gray1, None)
keypoints2, descriptors2 = sift.detectAndCompute(gray2, None)

# Match keypoints using FLANN matcher
matcher = cv2.BFMatcher()
matches = matcher.knnMatch(descriptors1, descriptors2, k=2) 
# matcher = cv2.FlannBasedMatcher()
# matches = matcher.knnMatch(descriptors1, descriptors2, k=2)

# Apply ratio test to filter good matches
good_matches = []
for m, n in matches:
    if m.distance < 0.8 * n.distance:
        good_matches.append(m)

# Estimate homography matrix
if len(good_matches) >= 4:
    src_pts = np.float32([keypoints1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([keypoints2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    homography, _ = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)

    # Warp image1 to image2 using homography
    stitched_image = cv2.warpPerspective(image2, homography, (image1.shape[1] + image2.shape[1], image1.shape[0]))
    cv2.imshow("Stitched Image", stitched_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    stitched_image[:, :image1.shape[1]] = image1
    result = stitched_image
else:
    print("Try different image matching parameters")
    raise

# Display result
cv2.imwrite("result/stitched_img.png", result)
cv2.imshow("Stitched Image", result)
cv2.waitKey(0)
cv2.destroyAllWindows()