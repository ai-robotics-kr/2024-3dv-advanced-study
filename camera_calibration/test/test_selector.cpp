#include "gtest/gtest.h"
#include "frame/frame.hpp"
#include "feature/feature.hpp"
#include "selector/selector.hpp"


#include <cmath>
#include <string>
#include <opencv2/opencv.hpp>



class PixelSelectorTest : public ::testing::Test {
protected:
    std::string data_path = "../data/left/frame0000.png";
    Frame frame = Frame(0, data_path);
};

TEST_F(PixelSelectorTest, TestShowFrame)
{
    cv::imshow("frame", frame.image);
    cv::waitKey(0);
}

TEST_F(PixelSelectorTest, TestSelect)
{
    PixelSelector pixel_selector;
    DepthPointGrid depth_point_grid = pixel_selector.select(frame);
    cv::Mat image = frame.image.clone();

    for (const auto& depth_point : depth_point_grid)
    {
        cv::circle(image, depth_point.point, 1, cv::Scalar(255, 255, 255), -1); 
    }



    cv::imshow("selected", image);
    cv::waitKey(0);
}