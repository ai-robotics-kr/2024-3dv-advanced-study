#include "gtest/gtest.h"

#include "depth_estimator/depth_estimator.hpp"
#include "util.hpp"
#include "frame/frame.hpp"
#include "feature/feature.hpp"
#include "selector/selector.hpp"


#include <cmath>
#include <string>
#include <iostream>


class DepthEstimatorTest : public ::testing::Test {
protected:
    std::string l_data_path = "../data/left/frame0000.png";
    std::string r_data_path = "../data/right/frame0000.png";
    std::string calib_path = "../data/stereo_calibration_result.json";

    Frame l_frame = Frame(0, l_data_path);
    Frame r_frame = Frame(0, r_data_path);

    StereoCamera stereo_cam;
    bool success = read_stereo_calib_result(calib_path, stereo_cam);

    PixelSelector pixel_selector;
    DepthPointGrid depth_point_grid = pixel_selector.select(l_frame);    
};

TEST_F(DepthEstimatorTest, TestShowFrame)
{
    // stack left and right image
    cv::Mat stacked;
    cv::hconcat(l_frame.image, r_frame.image, stacked);
    cv::imshow("left right frame", stacked);
    cv::waitKey(0);
}

TEST_F(DepthEstimatorTest, TestSelect)
{
    cv::Mat image = l_frame.image.clone(); cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);

    for (const auto& depth_point : depth_point_grid)
    {
        cv::circle(image, depth_point.point, 1, cv::Scalar(0, 0, 255), -1); 
    }

    cv::imshow("selected", image);
    cv::waitKey(0);
}



/// @brief Let's see the epipolar curve of the selected point (5, 5) 
TEST_F(DepthEstimatorTest, DrawEpipolarCurveOnePoint)
{
    DepthPoint depth_point = depth_point_grid.at(15, 15);
    
    cv::Mat l_image = l_frame.image.clone(); cv::cvtColor(l_image, l_image, cv::COLOR_GRAY2BGR);
    cv::circle(l_image, depth_point.point, 1, cv::Scalar(0, 0, 255), 5);

    // Get candidator
    cv::Point2d candidator_2d = depth_point.point;
    
    // Let it be in unit sphere
    cv::Point3d candidator_3d = stereo_cam.l_unproject(candidator_2d);

    // We will make candidator through the ray
    cv::Point3d max_point = candidator_3d / 0.1;
    cv::Point3d min_point = candidator_3d / 10.0;

    // Let them be in right camera coordinate
    cv::Point3d max_point_r = stereo_cam.transform_left_to_right(max_point);
    cv::Point3d min_point_r = stereo_cam.transform_left_to_right(min_point);

    // Let them be in unit sphere
    max_point_r = max_point_r / cv::norm(max_point_r);
    min_point_r = min_point_r / cv::norm(min_point_r);

    // Let's find candidators in right image
    std::vector<cv::Point3d> candidators_3d_r = unit_sphere_linspace(min_point_r, max_point_r, 100);

    std::cout << "candidators_3d_r.size() = " << candidators_3d_r.size() << std::endl;

    // Let them in right image
    std::vector<cv::Point2d> candidators_2d_r;
    for (int i = 0; i < candidators_3d_r.size(); i++)
    {
        cv::Point2d candidator_2d_r = stereo_cam.r_project(candidators_3d_r[i]);
        // is in the image?
        if (is_inside(candidator_2d_r, r_frame.image.size()))
            candidators_2d_r.push_back(candidator_2d_r);
    }
        

    // Let's draw the epipolar curve
    cv::Mat r_image = r_frame.image.clone(); cv::cvtColor(r_image, r_image, cv::COLOR_GRAY2BGR);
    for (const auto& candidator_2d_r : candidators_2d_r)
        cv::circle(r_image, candidator_2d_r, 1, cv::Scalar(0, 0, 255), 3);

    
    // Stack the left and right image
    cv::Mat stacked;
    cv::hconcat(l_image, r_image, stacked);
    cv::imshow("epipolar curve", stacked);
    cv::waitKey(0);
}



TEST_F(DepthEstimatorTest, DrawEpipolarCurvesForAllPoints)
{
    // 이미지를 BGRA 형식으로 변환
    cv::Mat l_image = l_frame.image.clone(); cv::cvtColor(l_image, l_image, cv::COLOR_GRAY2BGRA);
    cv::Mat r_image = r_frame.image.clone(); cv::cvtColor(r_image, r_image, cv::COLOR_GRAY2BGRA);

    for (int y = 0; y < depth_point_grid.rows(); y++)
    {
        for (int x = 0; x < depth_point_grid.cols(); x++)
        {
            DepthPoint depth_point = depth_point_grid.at(y, x);

            // 점마다 고유한 색상 생성
            cv::Scalar unique_color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);

            // 왼쪽 이미지에 점 그리기
            cv::circle(l_image, depth_point.point, 1, unique_color, 1);

            // 에피폴라 곡선 그리기를 위한 준비
            cv::Point2d candidator_2d = depth_point.point;
            cv::Point3d candidator_3d = stereo_cam.l_unproject(candidator_2d);
            cv::Point3d max_point = candidator_3d / 0.1;
            cv::Point3d min_point = candidator_3d / 10.0;
            cv::Point3d max_point_r = stereo_cam.transform_left_to_right(max_point);
            cv::Point3d min_point_r = stereo_cam.transform_left_to_right(min_point);
            max_point_r = max_point_r / cv::norm(max_point_r);
            min_point_r = min_point_r / cv::norm(min_point_r);
            std::vector<cv::Point3d> candidators_3d_r = unit_sphere_linspace(min_point_r, max_point_r, 100);

            std::vector<cv::Point2d> candidators_2d_r;
            for (int i = 0; i < candidators_3d_r.size(); i++)
            {
                cv::Point2d candidator_2d_r = stereo_cam.r_project(candidators_3d_r[i]);
                if (is_inside(candidator_2d_r, r_frame.image.size()))
                    candidators_2d_r.push_back(candidator_2d_r);
            }

            for (const auto& candidator_2d_r : candidators_2d_r)
                cv::circle(r_image, candidator_2d_r, 1, unique_color, 1);
        }
    }

    cv::Mat stacked;
    cv::hconcat(l_image, r_image, stacked);
    cv::imshow("epipolar curves", stacked);
    cv::waitKey(0);
}




TEST_F(DepthEstimatorTest, TestMatching)
{
    DepthPointGrid r_points = stereo_cam.simple_direct_match(
        l_frame, r_frame, depth_point_grid, 100);

    cv::Mat l_image = l_frame.image.clone(); cv::cvtColor(l_image, l_image, cv::COLOR_GRAY2BGR);
    cv::Mat r_image = r_frame.image.clone(); cv::cvtColor(r_image, r_image, cv::COLOR_GRAY2BGR);

    for (const auto& depth_point : depth_point_grid)
    {
        cv::circle(l_image, depth_point.point, 1, cv::Scalar(0, 0, 255), -1); 
    }

    for (const auto& depth_point : r_points)
    {
        cv::circle(r_image, depth_point.point, 1, cv::Scalar(0, 0, 255), -1); 
    }

    // stack left and right image
    cv::Mat stacked;
    cv::hconcat(l_image, r_image, stacked);
    cv::imshow("matched", stacked);
    cv::waitKey(0);
}