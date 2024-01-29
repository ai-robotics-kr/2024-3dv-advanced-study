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

class DoubleSphere : public CameraModel
{
public: 
    DoubleSphere() : CameraModel() {}
    explicit DoubleSphere(const std::vector<double>& params) : CameraModel()
    {
        if (params.size() != 6) {
            throw std::invalid_argument("DoubleSphere should have 5 parameters, fx, fy, cx, cy, xi, alpha");
        }        
        params_ = params;
    }

    void init_params(const double& f, const double& cx, const double& cy) noexcept override
    {
        params_ = { f, f, cx, cy, 0.5, 0.01 };
    }

    cv::Point2d project(const cv::Point3d& point) const noexcept override
    {
        const double fx = params_[0];
        const double fy = params_[1];
        const double cx = params_[2];
        const double cy = params_[3];
        const double xi = params_[4];
        const double alpha = params_[5];

        const double x = point.x;
        const double y = point.y;
        const double z = point.z;

        const double d1 = sqrt(x * x + y * y + z * z);
        const double d2 = sqrt(x * x + y * y + (xi * d1 + z) * (xi * d1 + z));

        const double tmp = alpha * d2 + (1.0 - alpha) * (xi * d1 + z);

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
        const double xi = params_[4];
        const double alpha = params_[5];

        const double mx = (pix.x - cx) / fx;
        const double my = (pix.y - cy) / fy;

        const double r2 = mx * mx + my * my;

        const double mz1 = 1 - alpha * alpha * r2;
        const double mz2 = alpha * sqrt(1.0 - (2 * alpha - 1.0) * r2) + 1.0 - alpha;
        const double mz = mz1 / mz2;

        const double tmp1 = mz * xi + sqrt( mz * mz + (1.0 - xi * xi) * r2);
        const double tmp2 = mz * mz + r2;

        const double x = mx * tmp1 / tmp2;
        const double y = my * tmp1 / tmp2;
        const double z = mz * tmp1 / tmp2 - xi;

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
            T xi = intrinsic_[4];
            T alpha = intrinsic_[5];

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

            T d1 = ceres::sqrt(x * x + y * y + z * z);
            T d2 = ceres::sqrt(x * x + y * y + (xi * d1 + z) * (xi * d1 + z));

            T tmp = alpha * d2 + (one - alpha) * (xi * d1 + z);


            T u = fx * x / tmp + cx;
            T v = fy * y / tmp + cy;

            residuals[0] = u - T(observed_.x);
            residuals[1] = v - T(observed_.y);

            return true;
        }

        static ceres::CostFunction* create(const cv::Point2d& obs, const cv::Point3d& pt)
        {
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 6>(new ReprojectionError(obs, pt)));
        }

    };

};




