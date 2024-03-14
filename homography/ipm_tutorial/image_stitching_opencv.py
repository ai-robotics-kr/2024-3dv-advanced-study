import cv2

image1 = cv2.imread("data/img1.png")
image2 = cv2.imread("data/img2.png")

stitcher = cv2.Stitcher.create(mode = 0) # 0 for pano (perspective), 1 for scan (affine)
result = stitcher.stitch((image1,image2))

cv2.imshow("Stitched Image", result[1])
cv2.waitKey(0)

stitcher = cv2.Stitcher.create(mode = 1)
result = stitcher.stitch((image1,image2))

cv2.imshow("Stitched Image", result[1])
cv2.waitKey(0)
cv2.destroyAllWindows()