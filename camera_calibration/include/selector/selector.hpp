#pragma once

#include "feature/feature.hpp"
#include "frame/frame.hpp"




struct SettingPixelSelector
{
    static constexpr double kMinGrad2   = 100.0;
    static constexpr double kMaxGrad2   = 200.0;
    static constexpr int    kCellWidth  = 10;
    static constexpr int    kCellHeight = 20;
};



class PixelSelector
{
private:
    SettingPixelSelector cfg_;

public:
    PixelSelector(SettingPixelSelector cfg = SettingPixelSelector()) 
        : cfg_(cfg) {}


    DepthPointGrid select(const Frame& frame) noexcept;
};


DepthPointGrid PixelSelector::select(const Frame& frame) noexcept
{
    int width = frame.width;
    int height = frame.height;

    int grid_width  = width / cfg_.kCellWidth;
    int grid_height = height / cfg_.kCellHeight;
    
    DepthPointGrid depth_point_grid(grid_height, grid_width);

    for (int r = 0; r < grid_height; r++) for (int c = 0; c < grid_width; c++)
    {
        double max_grad2 = -1.0;

        for (int dr = 0; dr < cfg_.kCellHeight; dr++) for (int dc = 0; dc < cfg_.kCellWidth; dc++)
        {
            double y = static_cast<double> (r * cfg_.kCellHeight + dr);
            double x = static_cast<double> (c * cfg_.kCellWidth + dc);

            if (is_inside(cv::Point2d(x, y), frame.image.size()))
            {
                if (max_grad2 > cfg_.kMaxGrad2) continue;

                double grad2 = frame.get_gradient({x, y});
                if (grad2 < cfg_.kMinGrad2) continue;

                max_grad2 = grad2;

                depth_point_grid.at(r, c).set_pix({x, y});
            }
        }
    }

    return depth_point_grid;
}