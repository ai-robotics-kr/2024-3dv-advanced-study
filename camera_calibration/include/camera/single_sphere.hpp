#pragma once

#include "camera/camera_model_template.hpp"
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <limits>
#include <cmath>
#include <iostream>
#include <vector>



using std::atan2;
using std::sqrt;

class SingleSphere : public CameraModel
{
public: 
    SingleSphere() : CameraModel() {}
    explicit SingleSphere(const std::vector<double>& params) : CameraModel()
    {
        if (params.size() != 5) {
            throw std::invalid_argument("SingleSphere should have 4 parameters, fx, fy, cx, cy, alpha");
        }        
        params_ = params;
    }

    void init_params(const double& f, const double& cx, const double& cy) noexcept override
    {
        params_ = { f, f, cx, cy, 0.5 };
    }

    cv::Point2d project(const cv::Point3d& point) const noexcept override
    {
        const double fx = params_[0];
        const double fy = params_[1];
        const double cx = params_[2];
        const double cy = params_[3];
        const double alpha = params_[4];

        const double x = point.x;
        const double y = point.y;
        const double z = point.z;

        const double d = sqrt(x * x + y * y + z * z);
        const double tmp = alpha * d + (1.0 - alpha) * z;

        const double u = fx * x / tmp + cx;
        const double v = fy * y / tmp + cy;

        return cv::Point2d(u, v);
    }

    cv::Point3d unproject(const cv::Point2d& pix) const noexcept override
    {
        const double fx = params_[0];
        const double fy = params_[1];
        const double cx = params_[2];
        const double cy = params_[3];
        const double alpha = params_[4];

        const double mx = (pix.x - cx) / fx * (1 - alpha);
        const double my = (pix.y - cy) / fy * (1 - alpha);

        const double r2 = mx * mx + my * my;
        const double delta = alpha / (1.0 - alpha);

        const double tmp1 = delta + sqrt (1.0 + (1.0 - delta * delta) * r2);
        const double tmp2 = 1 + r2;

        const double x = mx * tmp1 / tmp2;
        const double y = my * tmp1 / tmp2;
        const double z = tmp1 / tmp2 - delta;

        return cv::Point3d(x, y, z);
    }

    struct ReprojectionError
    {
        cv::Point2d observed_;
        cv::Point3d point_;

        ReprojectionError(cv::Point2d observed, cv::Point3d point): observed_(observed), point_(point) {}

        template <typename T>
        bool operator() (const T* const intrinsic_, const T* const transform, T* residuals) const
        {
            T fx = intrinsic_[0];
            T fy = intrinsic_[1];
            T cx = intrinsic_[2];
            T cy = intrinsic_[3];
            T alpha = intrinsic_[4];

            T P[3];
            T point[3] = {T(point_.x), T(point_.y), T(point_.z)};

            ceres::AngleAxisRotatePoint(transform, point, P);

            P[0] += transform[3];
            P[1] += transform[4];
            P[2] += transform[5];

            T x = P[0];
            T y = P[1];
            T z = P[2];
            T one = T(1);

            T d = ceres::sqrt(x * x + y * y + z * z);
            T tmp = alpha * d + (one - alpha) * z;

            T u = fx * x / tmp + cx;
            T v = fy * y / tmp + cy;

            residuals[0] = u - T(observed_.x);
            residuals[1] = v - T(observed_.y);

            return true;
        }

        static ceres::CostFunction* create(const cv::Point2d& obs, const cv::Point3d& pt)
        {
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 5, 6>(new ReprojectionError(obs, pt)));
        }

    };

};



