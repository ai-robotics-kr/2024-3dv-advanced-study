#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>






struct Frame
{
    explicit Frame(size_t id_, const std::string image_path) 
        :  id(id_)
        {
            image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
            width = image.cols;
            height = image.rows;
            cv::Sobel(image, grad_x, CV_64F, 1, 0, 3);
            cv::Sobel(image, grad_y, CV_64F, 0, 1, 3);
        }

    std::vector<cv::Point2d> get_corners() const noexcept { return corners; }
    void set_corners(const std::vector<cv::Point2d>& corners_) noexcept { corners = corners_; }
    
    cv::Point2d get_corner(const int& id) const noexcept { return corners[id]; }
    int get_corner_num() const noexcept { return corners.size(); }
    
    cv::Vec6d get_transform() const noexcept { return transform; }
    void set_transform(const cv::Vec6d& transform_) noexcept { transform = transform_; }
    
    uchar at(const cv::Point2d& point) const noexcept { return image.at<uchar>(point); }
    double intensity_at(const cv::Point2d& pt) const noexcept;
    
    double get_gradient(const cv::Point2d& point) const noexcept
    { return std::sqrt(std::pow(grad_x.at<double>(point), 2) + std::pow(grad_y.at<double>(point), 2)); }


    size_t                      id;
    int                         width;
    int                         height;
    cv::Mat                     image;
    cv::Mat                     grad_x;
    cv::Mat                     grad_y;
    std::vector<cv::Point2d>    corners;
    cv::Vec6d                   transform;
};


// bilinear interpolation
double Frame::intensity_at(const cv::Point2d& pt) const noexcept
{
    if (pt.x < 0 || pt.y < 0 || pt.x >= width || pt.y >= height) return 0.0;
    int x = static_cast<int>(pt.x);
    int y = static_cast<int>(pt.y);
    double dx = pt.x - x;
    double dy = pt.y - y;
    double intensity = (1 - dx) * (1 - dy) * at(cv::Point2i(x, y)) +
                       dx * (1 - dy) * at(cv::Point2i(x + 1, y)) +
                       (1 - dx) * dy * at(cv::Point2i(x, y + 1)) +
                       dx * dy * at(cv::Point2i(x + 1, y + 1));
    return intensity;
}



bool is_inside(const cv::Point2d& point, const cv::Size& size) noexcept
{
    return point.x >= 0 && point.x < size.width && point.y >= 0 && point.y < size.height;
}

typedef std::vector<Frame> ImagePyramid;