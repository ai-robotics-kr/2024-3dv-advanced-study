import cv2
import numpy as np
from utils import perspective, Plane, bilinear_sampler

import matplotlib.pyplot as plt


TARGET_H, TARGET_W = 500, 500

def ipm_from_parameters(image, xyz, K, RT, interpolation_fn):
    # Flip y points positive upwards
    xyz[1] = -xyz[1]
    P = K @ RT
    pixel_coords = perspective(xyz, P, TARGET_H, TARGET_W)
    image2 = interpolation_fn(image, pixel_coords)
    return image2.astype(np.uint8)

def ipm_from_opencv(image, source_points, target_points):
    M = cv2.getPerspectiveTransform(source_points, target_points)
    warped = cv2.warpPerspective(image, M, (TARGET_W, TARGET_H), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return warped

if __name__ == '__main__':
    # Load the image
    image = cv2.cvtColor(cv2.imread('data/cityscape_sample.png'), cv2.COLOR_BGR2RGB)
    image_vis = cv2.resize(image, (1024, 512)) 
    cv2.imshow("Perspective view", image_vis)
    cv2.waitKey(0)

    # Define the plane on the region of interest (road)
    plane = Plane(0, -25, 0, 0, 0, 0, TARGET_H, TARGET_W, 0.1)
    # Retrieve camera parameters: World-2-Camera of Cityscapes dataset
    extrinsic = np.array([[-9.72664662e-03, -9.99952695e-01,  0.00000000e+00,  4.27740580e-02],
                          [-3.84143274e-02,  3.73660263e-04, -9.99261827e-01,  1.27679959e+00],
                          [ 9.99214557e-01, -9.71946668e-03, -3.84161446e-02, -1.65183398e+00],
                          [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
    intrinsic = np.array([[2.26354773e+03, 0.00000000e+00, 1.07901756e+03, 0.00000000e+00],
                          [0.00000000e+00, 2.25037282e+03, 5.15006601e+02, 0.00000000e+00],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00, 0.00000000e+00],
                          [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    
    # Apply perspective transformation
    warped_raw = ipm_from_parameters(image, plane.xyz, intrinsic, extrinsic, bilinear_sampler)

    cv2.imwrite("result/IPM.png", warped_raw)
    cv2.imshow("IPM", warped_raw)
    cv2.waitKey(0)

    # Vertices coordinates in the source image
    s = np.array([[830, 598],
                  [868, 568],
                  [1285, 598],
                  [1248, 567]], dtype=np.float32)

    # Vertices coordinates in the destination image
    t = np.array([[177, 231],
                  [213, 231],
                  [178, 264],
                  [216, 264]], dtype=np.float32)

    # Warp the image
    warped2 = ipm_from_opencv(image, s, t)
    

    # Draw results
    cv2.imwrite("result/Opencv_IPM.png", warped2)
    cv2.imshow("Opencv_IPM", warped2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
