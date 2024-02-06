#pragma once

#include "frame/frame.hpp"
#include "grid.hpp"


#include <opencv2/core/core.hpp>
#include <vector>
#include <array>
#include <iostream>




struct SettingPoint
{
    static constexpr double kMinIDepth = 0.1;
    static constexpr double kMaxIDepth = 10.0;
    static constexpr double kBadIDepth = -1.0;
    static constexpr double kNand = std::numeric_limits<double>::quiet_NaN();
    inline static const cv::Point2d kBadPixD = {kNand, kNand};
};


struct DepthPoint
{
    SettingPoint cfg;
    cv::Point2d point;
    double idepth;
    // double jac;

    DepthPoint(SettingPoint cfg = SettingPoint())
        : cfg(cfg), point(cfg.kBadPixD), idepth(cfg.kBadIDepth) {}

    void set_pix(const cv::Point2d& point_) { point = point_; }
    void set_idepth(const double& idepth_) { idepth = idepth_; }
    void update_idepth(const double d_idepth) { idepth = std::max(0.0, idepth + d_idepth); }
};


struct Patch
{
    using Point2dArray = std::array<cv::Point2d, 9>;
    using ArrayKd = std::array<double, 9>;
    
    inline static const Point2dArray kOffsetPx = {
        cv::Point2d{0, 0}, {0, -2}, {-1, -1}, {1, -1}, {-2, 0}, {2, 0}, {-1, 1}, {1, 1}, {0, 2}
    };

    ArrayKd intensity_vals{};
    ArrayKd grads{};

    // Extract intensity and gradient from gray image at patch pxs
    void extract(const Frame& image, const Point2dArray& point)
    {
        for (int i = 0; i < 9; i++)
        {
            intensity_vals[i] = image.at(point[i]);
            grads[i] = image.get_gradient(point[i]);
        }
    }
    bool is_any_out(const Frame& mat, const cv::Point2d& cpoint) noexcept
    {
        for (const auto& point : kOffsetPx)
        {
            cv::Point2d p = cpoint + point;
            if (p.x < 0.0 || p.y < 0.0 || p.x >= mat.width || p.y >= mat.height) return true;
        }
        return false;
    }
};

using PatchGrid = Grid2d<Patch>;
using PixelGrid = Grid2d<cv::Point2i>;
using DepthPointGrid = Grid2d<DepthPoint>;