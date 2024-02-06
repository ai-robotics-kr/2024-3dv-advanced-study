#pragma once

#include "board/board.hpp"
#include "frame/frame.hpp"
#include "camera/cameras.hpp"
#include <opencv2/core/core.hpp>
#include <vector>
#include <array>








class ChessBoard : public BaseBoard
{
public:
    explicit ChessBoard(cv::Size board_pattern, std::array<double, 2> board_size) : BaseBoard(board_pattern, board_size) {}

    /// @brief If find corners, set corners into frame and return true, otherwise return false.
    bool find_corners(Frame& frame) const noexcept override
    {
        std::vector<cv::Point2f> corners;

        bool found = cv::findChessboardCorners(frame.image, board_pattern_, corners);

        if (found)
        {
            cv::cornerSubPix(frame.image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            // float to double
            std::vector<cv::Point2d> corners_d;
            corners_d.reserve(corners.size());
            for (const auto& corner : corners)
                corners_d.emplace_back(corner.x, corner.y);
            
            frame.set_corners(corners_d);
        }

        return found;
    }

    // For finding initial pose, we use solvePnP.
    bool find_initial_pose(CameraModel& cam, Frame& frame) const noexcept override
    {
        std::vector<double> param = cam.get_params();
        double fx = param[0];
        double fy = param[1];
        double cx = param[2];
        double cy = param[3];

        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        cv::Mat distortion = (cv::Mat_<double>(1, 4) << 0.0, 0.0, 0.0, 0.0);
        cv::Mat rvec, tvec;

        std::vector<cv::Point2d> corners = frame.get_corners();
        std::vector<cv::Point2f> corners_f;

        // double to float
        corners_f.reserve(corners.size());
        for (const auto& corner : corners)
            corners_f.emplace_back(corner.x, corner.y);
        
        
        bool success = cv::solvePnP(board_points_, corners_f, camera_matrix, distortion, rvec, tvec);

        if (success)
        {
            cv::Vec6d transform;
            transform[0] = rvec.at<double>(0, 0);
            transform[1] = rvec.at<double>(1, 0);
            transform[2] = rvec.at<double>(2, 0);
            transform[3] = tvec.at<double>(0, 0);
            transform[4] = tvec.at<double>(1, 0);
            transform[5] = tvec.at<double>(2, 0);

            frame.set_transform(transform);

            return true;
        }

        return false;
    }
};



