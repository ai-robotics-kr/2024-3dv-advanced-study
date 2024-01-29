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

class TripleSphere : public CameraModel
{
public: 
    TripleSphere() : CameraModel() {}
    explicit TripleSphere(const std::vector<double>& params) : CameraModel()
    {
        if (params.size() != 7) {
            throw std::invalid_argument("TripleSphere should have 4 parameters, fx, fy, cx, cy, xi, lambda, alpha");
        }        
        params_ = params;
    }

    void init_params(const double& f, const double& cx, const double& cy) noexcept override
    {
        params_ = { f, f, cx, cy, 0.5, 0.5, 0.5 };
    }

    cv::Point2d project(const cv::Point3d& point) const noexcept override
    {
        const double fx = params_[0];
        const double fy = params_[1];
        const double cx = params_[2];
        const double cy = params_[3];
        const double xi = params_[4];
        const double lambda = params_[5];
        const double alpha = params_[6];

        const double x = point.x;
        const double y = point.y;
        const double z = point.z;

        const double d1 = sqrt(x * x + y * y + z * z);
        const double d2 = sqrt(x * x + y * y + (xi * d1 + z) * (xi * d1 + z));
        const double d3 = sqrt(x * x + y * y + (lambda * d2 + xi * d1 + z) * (lambda * d2 + xi * d1 + z));

        const double zeta = (1.0 - alpha) * (z + xi * d1 + lambda * d2) + alpha * d3;

        const double u = fx * x / zeta + cx;
        const double v = fy * y / zeta + cy;

        return cv::Point2d(u, v);
    }

    cv::Point3d unproject(const cv::Point2d& pix) const noexcept override
    {
        const double fx = params_[0];
        const double fy = params_[1];
        const double cx = params_[2];
        const double cy = params_[3];
        const double xi = params_[4];
        const double lambda = params_[5];
        const double alpha = params_[6];

        const double mx = (pix.x - cx) / fx;
        const double my = (pix.y - cy) / fy;

        const double zeta = alpha <= 0.5 ? alpha / (1.0 - alpha) : (1.0 - alpha) / alpha;
        
        const double gamma1 = zeta + sqrt(1 + (1 - zeta * zeta) * (mx * mx + my * my));
        const double gamma2 = mx * mx + my * my + 1;
        const double gamma = gamma1 / gamma2;

        const double nu = lambda * (gamma - zeta) + sqrt(lambda * lambda * (gamma - zeta) * (gamma - zeta) - lambda * lambda + 1.0);

        const double mz = nu * (gamma - zeta) - lambda;

        const double mu = xi * mz + sqrt(xi * xi * mz * mz - xi * xi + 1.0);

        const double x = mu * nu * gamma * mx;
        const double y = mu * nu * gamma * my;
        const double z = mu * mz - xi;

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
            T lambda = intrinsic_[5];
            T alpha = intrinsic_[6];

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

            T d1 = sqrt(x * x + y * y + z * z);
            T d2 = sqrt(x * x + y * y + (xi * d1 + z) * (xi * d1 + z));
            T d3 = sqrt(x * x + y * y + (lambda * d2 + xi * d1 + z) * (lambda * d2 + xi * d1 + z));

            // T zeta = z + xi * d1 + lambda * d2 + alpha / (one - alpha) * d3;
            T zeta =  (one - alpha) * (z + xi * d1 + lambda * d2)+ alpha * d3;

            T u = fx * x / zeta + cx;
            T v = fy * y / zeta + cy;

            residuals[0] = u - T(observed_.x);
            residuals[1] = v - T(observed_.y);

            return true;
        }

        static ceres::CostFunction* create(const cv::Point2d& obs, const cv::Point3d& pt)
        {
            return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 7, 6>(new ReprojectionError(obs, pt)));
        }

    };

};




