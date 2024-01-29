/**
 * File: camera_model_template.hpp
 * Author: Jun Hyeok Choi, dkwnsgur12@gmail.com
 * Date: 2023-12-27
 * Copyright (c) 2023 Jun Hyeok Choi. All rights reserved.
 * Description: This file is for camera model template. All camera model should be derived from this class.
 * 
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <vector>




/// @brief This class is for camera model template.
class CameraModel
{
public:
    CameraModel() = default;
    virtual ~CameraModel() {}

    virtual void init_params(const double& f, const double& cx, const double& cy) noexcept = 0;
    virtual cv::Point2d project(const cv::Point3d& point) const noexcept = 0;
    virtual cv::Point3d unproject(const cv::Point2d& pixel) const noexcept = 0;

    std::vector<double> get_params() const noexcept { return params_; }
    // const std::vector<double>& params() const { return params_; }
    void set_params(const std::vector<double>& params) noexcept { params_ = params; }

    struct ReprojectionError
    {
        ReprojectionError(const cv::Point2d& pixel, const cv::Point3d& point) : pixel_(pixel), point_(point) {}

        template <typename T>
        bool operator()(const T* const camera_params, const T* const transform, T* residuals) const
        {}

        static void create(const cv::Point2d& pixel, const cv::Point3d& point) {}
        const cv::Point2d pixel_;
        const cv::Point3d point_;
    };
    

protected:
    std::vector<double> params_;
};



