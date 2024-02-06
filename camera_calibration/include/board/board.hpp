#pragma once

#include "frame/frame.hpp"
#include "camera/cameras.hpp"
#include <opencv2/core/core.hpp>
#include <vector>
#include <array>








class BaseBoard
{
public:
    explicit BaseBoard(cv::Size board_pattern, std::array<double, 2> board_size) : board_pattern_(board_pattern), board_size_(board_size)
    {
        if (board_pattern_.width <= 0 || board_pattern_.height <= 0)
            throw std::invalid_argument("board_pattern should be positive");
        if (board_size_[0] <= 0 || board_size_[1] <= 0) 
            throw std::invalid_argument("board_size should be positive");
        
        board_points_.reserve(board_pattern_.width * board_pattern_.height);
        
        double h = board_size_[0];
        double w = board_size_[1];

        for (int i = 0; i < board_pattern_.height; i++)
        for (int j = 0; j < board_pattern_.width; j++)
            board_points_.emplace_back(j * w, i * h, 0.0);

    }

    virtual bool find_corners(Frame& frame) const noexcept = 0;
    virtual bool find_initial_pose(CameraModel& cam, Frame& frame) const noexcept = 0;

    void show_board_info() const noexcept
    {
        std::cout << "board_pattern: " << board_pattern_ << std::endl;
        std::cout << "board_size: " << board_size_[0] << " " << board_size_[1] << std::endl;

        std::cout << "board_points: " << std::endl;
        for (const auto& point : board_points_)
            std::cout << point << std::endl;
    }

    cv::Point3d get_board_point(const int& id) const noexcept { return board_points_[id]; }

protected:
    cv::Size board_pattern_;
    std::array<double, 2> board_size_;
    std::vector<cv::Point3d> board_points_;
};


