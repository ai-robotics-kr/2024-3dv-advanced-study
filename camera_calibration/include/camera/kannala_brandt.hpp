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
using std::tan;
using std::sqrt;


class KannalaBrandt : public CameraModel
{

public:
    KannalaBrandt() : CameraModel() {}
    explicit KannalaBrandt(const std::vector<double> &params) : CameraModel()
    {
        if (params.size() != 8)
        {
            throw std::invalid_argument("KannalaBrandt should have 8 parameters, fx, fy, cx, cy, k1, k2, k3, k4");
        }
        params_ = params;
    }

    void init_params(const double& f, const double& cx, const double& cy) noexcept override
    {
        params_ = { f, f, cx, cy, -0.2, 0.1, 0.0, 0.0 };
    }

    cv::Point2d project(const cv::Point3d& point) const noexcept override
    {
        const double fx = params_[0];
        const double fy = params_[1];
        const double cx = params_[2];
        const double cy = params_[3];
        const double k1 = params_[4];
        const double k2 = params_[5];
        const double k3 = params_[6];
        const double k4 = params_[7];

        const double x = point.x / point.z;
        const double y = point.y / point.z;

        const double r = sqrt(x * x + y * y);
        const double theta = atan2(r, 1.0);

        const double theta2 = theta * theta;
        const double theta4 = theta2 * theta2;
        const double theta6 = theta4 * theta2;
        const double theta8 = theta6 * theta2;

        const double theta_d = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8) / r;

        const double u = fx * theta_d * x + cx;
        const double v = fy * theta_d * y + cy;

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
        const double k3 = params_[6];
        const double k4 = params_[7];

        const double mx = (pix.x - cx) / fx;
        const double my = (pix.y - cy) / fy;

        const double theta_d = sqrt(mx * mx + my * my);
        const double theta = gauss_newton(theta_d, k1, k2, k3, k4);

        const double x = mx * tan(theta) / theta_d;
        const double y = my * tan(theta) / theta_d;

        return cv::Point3d(x, y, 1.0);
    }

    double gauss_newton(double rd, double k1, double k2, double k3, double k4) const noexcept
    {
        double ru = rd;
        double init_diff = std::numeric_limits<double>::max();
        double step = 0.1;
        size_t max_i = 100;

        for (int i = 0; i < max_i; i++)
        {
            const double ru2 = ru * ru;
            const double ru4 = ru2 * ru2;
            const double ru6 = ru4 * ru2;
            const double ru8 = ru6 * ru2;

            const double up = ru * (1.0 + k1 * ru2 + k2 * ru4 + k3 * ru6 + k4 * ru8);
            const double down = 1.0 + 3.0 * k1 * ru2 + 5.0 * k2 * ru4 + 7.0 * k3 * ru6 + 9.0 * k4 * ru8;
            const double diff = rd - up;

            if (init_diff > std::abs(diff)) step *= 1.2;
            else step *= -0.5;

            if (std::abs(diff) < 1e-7) break;

            init_diff = std::abs(diff);
            ru -= step * up / down;
        }
        std::cout << "init_diff: " << init_diff << std::endl;
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
            T k3 = intrinsic_[6];
            T k4 = intrinsic_[7];

            T P[3];
            T point[3] = {T(point_.x), T(point_.y), T(point_.z)};

            ceres::AngleAxisRotatePoint(transform, point, P);

            P[0] += transform[3];
            P[1] += transform[4];
            P[2] += transform[5];

            T x = P[0] / P[2];
            T y = P[1] / P[2];
            T one = T(1);

            T r = sqrt(x * x + y * y);
            T theta = atan2(r, one);
            T theta2 = theta * theta;
            T theta4 = theta2 * theta2;
            T theta6 = theta4 * theta2;
            T theta8 = theta6 * theta2;

            T radial = theta * (one + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8) / r;

            T u = fx * x * radial + cx;
            T v = fy * y * radial + cy;

            residuals[0] = u - T(observed_.x);
            residuals[1] = v - T(observed_.y);

            return true;
        }

        static ceres::CostFunction* create(const cv::Point2d& obs, const cv::Point3d& pt)
        {
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 8, 6>(new ReprojectionError(obs, pt)));
        }

    };

};




