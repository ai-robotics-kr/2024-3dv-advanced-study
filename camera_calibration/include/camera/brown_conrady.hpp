#pragma once

#include "camera/camera_model_template.hpp"
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <limits>
#include <cmath>
#include <iostream>
#include <vector>




class BrownConrady : public CameraModel
{
public:
    BrownConrady() : CameraModel() {}
    explicit BrownConrady(const std::vector<double>& params) : CameraModel()
    {
        if (params.size() != 6) {
            throw std::invalid_argument("BrownConrady should have 6 parameters, fx, fy, cx, cy, k1, k2");
        }        
        params_ = params;
    }

    void init_params(const double& f, const double& cx, const double& cy) noexcept override
    {
        params_ = { f, f, cx, cy, -0.2, 0.1 };
    }

    cv::Point2d project(const cv::Point3d& point) const noexcept override
    {
        const double fx = params_[0];
        const double fy = params_[1];
        const double cx = params_[2];
        const double cy = params_[3];
        const double k1 = params_[4];
        const double k2 = params_[5];

        const double x = point.x / point.z;
        const double y = point.y / point.z;

        const double r2 = x * x + y * y;
        const double r4 = r2 * r2;

        const double radial = 1.0 + k1 * r2 + k2 * r4;

        const double u = fx * x * radial + cx;
        const double v = fy * y * radial + cy;

        return cv::Point2d(u, v);
    }

    cv::Point3d unproject(const cv::Point2d& pix) const noexcept override
    {
        const double fx = params_[0];
        const double fy = params_[1];
        const double cx = params_[2];
        const double cy = params_[3];
        const double k1 = params_[4];
        const double k2 = params_[5];

        const double mx = (pix.x - cx) / fx;
        const double my = (pix.y - cy) / fy;

        const double rd = std::sqrt(mx * mx + my * my);
        double ru = gauss_newton(rd, k1, k2);

        const double x = mx * ru / rd;
        const double y = my * ru / rd;

        return cv::Point3d(x, y, 1.0);
    }


    double gauss_newton(double rd, double k1, double k2) const noexcept
    {
        double ru = rd;
        double init_diff = std::numeric_limits<double>::max();
        double step = 0.1;
        size_t max_i = 100;

        for (size_t i=0; i<max_i; i++)
        {
            const double ru2 = ru * ru;
            const double ru4 = ru2 * ru2;
            const double up = ru * (1 + k1 * ru2 + k2 * ru4);
            const double down = 1 + 3 * k1 * ru2 + 5 * k2 * ru4;
            const double diff = rd - up;

            if (init_diff > std::abs(diff)) step *= 1.2;
            else step *= -0.5;

            if (std::abs(diff) < 1e-7) break;

            init_diff = std::abs(diff);
            ru -= step * up / down;
        }
        return ru;
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
            T k1 = intrinsic_[4];
            T k2 = intrinsic_[5];

            T P[3];
            T point[3] = {T(point_.x), T(point_.y), T(point_.z)};

            ceres::AngleAxisRotatePoint(transform, point, P);

            P[0] += transform[3];
            P[1] += transform[4];
            P[2] += transform[5];

            T x = P[0] / P[2];
            T y = P[1] / P[2];
            T one = T(1);

            T r2 = x * x + y * y;
            T r4 = r2 * r2;

            T radial = one + k1 * r2 + k2 * r4;

            T u = fx * x * radial + cx;
            T v = fy * y * radial + cy;

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



