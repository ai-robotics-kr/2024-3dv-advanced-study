#include "camera/camera_model_template.hpp"
#include "camera/brown_conrady.hpp"
#include "camera/kannala_brandt.hpp"
#include "camera/single_sphere.hpp"
#include "camera/double_sphere.hpp"
#include "camera/triple_sphere.hpp"
#include "frame/frame.hpp"
#include "board/chessboard.hpp"
#include "util.hpp"

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <iostream>
#include <cmath>



int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Usage: ./undistort <path/to/image> <path/to/calib_result>" << std::endl;
        return 0;
    }

    std::string img_path = argv[1];

    std::cout << "Calibrate " << img_path << " image!\n";

    cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

    std::string calib_result = argv[2];

    std::cout << "Loading calibration result: " << calib_result << std::endl;

    CalibResult calib = read_calib_result(calib_result);

    std::cout << "Calibration result:\n";
    calib.show();


    std::unique_ptr<CameraModel> cam;

    if (calib.camera_model == "BrownConrady")
    {
        cam = std::make_unique<BrownConrady>();
    }
    else if (calib.camera_model == "KannalaBrandt")
    {
        cam = std::make_unique<KannalaBrandt>();
    }
    else if (calib.camera_model == "SingleSphere")
    {
        cam = std::make_unique<SingleSphere>();
    }
    else if (calib.camera_model == "DoubleSphere")
    {
        cam = std::make_unique<DoubleSphere>();
    }
    else if (calib.camera_model == "TripleSphere")
    {
        cam = std::make_unique<TripleSphere>();
    }
    else
    {
        std::cout << "Invalid camera model!\n";
        return 0;
    }

    cam->set_params(calib.parameters);

    cv::Mat undistorted_img = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);

    cv::Mat intrinsic = cv::Mat::eye(3, 3, CV_64F);
    intrinsic.at<double>(0, 0) = cam->get_params()[0];
    intrinsic.at<double>(1, 1) = cam->get_params()[1];
    intrinsic.at<double>(0, 2) = cam->get_params()[2];
    intrinsic.at<double>(1, 2) = cam->get_params()[3];

    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; ++j)
        {
            cv::Point2d pixel(j, i);
            cv::Point3d point = cam->unproject(pixel);

            cv::Point2d undistorted_pixel = cv::Point2d(
                intrinsic.at<double>(0, 0) * point.x / point.z + intrinsic.at<double>(0, 2),
                intrinsic.at<double>(1, 1) * point.y / point.z + intrinsic.at<double>(1, 2)
            );

            if (undistorted_pixel.x >= 0 && undistorted_pixel.x < img.cols && undistorted_pixel.y >= 0 && undistorted_pixel.y < img.rows)
            {
                undistorted_img.at<uchar>(undistorted_pixel.y, undistorted_pixel.x) = img.at<uchar>(i, j);
            }
        }
    }

    // 두 이미지 합치기
    cv::Mat result(img.rows, img.cols * 2, CV_8UC1);
    img.copyTo(result(cv::Rect(0, 0, img.cols, img.rows)));
    undistorted_img.copyTo(result(cv::Rect(img.cols, 0, img.cols, img.rows)));
    
    cv::imshow("result", result);
    cv::waitKey(0);

    return 0;
}