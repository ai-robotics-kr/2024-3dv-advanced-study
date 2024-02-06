#pragma once

#include "camera/camera_model_template.hpp"
#include <memory>
#include <vector>




class StereoCamera
{
using CameraModelPtr = std::shared_ptr<CameraModel>;

private:
    CameraModelPtr left_camera_;
    CameraModelPtr right_camera_;
    cv::Vec6d T_from_to_c2_c1_;

public:
    StereoCamera(CameraModelPtr l_cam, CameraModelPtr r_cam, const cv::Vec6d& T_from_to_c2_c1)
        : left_camera_(std::move(l_cam)), right_camera_(std::move(r_cam)), T_from_to_c2_c1_(T_from_to_c1_c2) {}
    
    std::vector<double> direct_match(const cv::Mat& l_img,
                                     const cv::Mat& r_img,
                                     const std::vector<cv::Point2d>& l_points) const noexcept;
};