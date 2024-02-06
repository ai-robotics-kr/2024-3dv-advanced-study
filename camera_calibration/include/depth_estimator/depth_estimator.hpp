#pragma once

#include "camera/cameras.hpp"
#include "frame/frame.hpp"
#include "feature/feature.hpp"

#include "util.hpp"

#include <memory>
#include <vector>









class StereoCamera
{
using CameraModelPtr = std::shared_ptr<CameraModel>;

private:
    CameraModelPtr left_camera_;
    CameraModelPtr right_camera_;
    cv::Vec6d T_from_to_c1_c2_;

public:
    StereoCamera() = default;
    StereoCamera(CameraModelPtr l_cam, CameraModelPtr r_cam, const cv::Vec6d& T_from_to_c1_c2)
        : left_camera_(std::move(l_cam)), right_camera_(std::move(r_cam)), T_from_to_c1_c2_(T_from_to_c1_c2) {}
    
    DepthPointGrid simple_direct_match(const Frame& l_img,
                                     const Frame& r_img,
                                     const DepthPointGrid& l_points,
                                     int n_candidators) noexcept;
    
    void set_left_cam(CameraModelPtr l_cam) noexcept { left_camera_ = std::move(l_cam); }
    void set_right_cam(CameraModelPtr r_cam) noexcept { right_camera_ = std::move(r_cam); }
    void set_T_from_to_c1_c2(const cv::Vec6d& T_from_to_c1_c2) noexcept { T_from_to_c1_c2_ = T_from_to_c1_c2; }

    cv::Mat Rmat() const noexcept
    {
        cv::Mat R;
        cv::Vec3d rvec{ T_from_to_c1_c2_[0], T_from_to_c1_c2_[1], T_from_to_c1_c2_[2] };
        cv::Rodrigues(rvec, R);
        return R;
    }

    cv::Mat tvec() const noexcept
    {
        cv::Mat t = (cv::Mat_<double>(3, 1) << T_from_to_c1_c2_[3], T_from_to_c1_c2_[4], T_from_to_c1_c2_[5]);
        return t;
    }

    cv::Point3d transform_left_to_right(cv::Point3d& point) const noexcept
    {
        cv::Mat R = Rmat();
        cv::Mat t = tvec();
        cv::Mat point_mat = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
        cv::Mat transformed_point = R * point_mat + t;
        
        return cv::Point3d(transformed_point.at<double>(0), transformed_point.at<double>(1), transformed_point.at<double>(2));
    }

    cv::Point2d l_project(cv::Point3d& point)   noexcept { return left_camera_->project(point); }
    cv::Point2d r_project(cv::Point3d& point)   noexcept { return right_camera_->project(point); }
    cv::Point3d l_unproject(cv::Point2d& point) noexcept { return left_camera_->unproject(point); }
    cv::Point3d r_unproject(cv::Point2d& point) noexcept { return right_camera_->unproject(point); }
};


DepthPointGrid StereoCamera::simple_direct_match(
    const Frame& l_img,
    const Frame& r_img,
    const DepthPointGrid& l_points,
    int n_candidators) noexcept
{
    std::array<cv::Point2d, 13> PATCH_OFFSET = {
        cv::Point2d{-2, -2}, {0, -2}, {2, -2},
        {-1, -1}, {1, -1},
        {-2, 0}, {0, 0}, {2, 0},
        {-1, 1}, {1, 1},
        {-2, 2}, {0, 2}, {2, 2}
    };

    DepthPointGrid r_points(l_points.rows(), l_points.cols());

    for (int i = 0; i < l_points.size(); i++)
    {
        cv::Point2d pt = l_points[i].point;

        // Extract patch and calc intensity
        double left_intensity = 0.0;
        for (const auto& offset: PATCH_OFFSET)
        {
            // cv::Point2d p = pt.point + offset;
            left_intensity += l_img.intensity_at(pt + offset);
        }
        left_intensity /= PATCH_OFFSET.size();

        // Let the point into right image with the epipolar curve
        cv::Point3d point_3d = l_unproject(pt);
        cv::Point3d min_cand = point_3d / 5.0;
        cv::Point3d max_cand = point_3d / 0.2;

        // Let them be in right camera coordinate
        cv::Point3d min_cand_r = transform_left_to_right(min_cand);
        cv::Point3d max_cand_r = transform_left_to_right(max_cand);

        // Let them be in unit sphere
        min_cand_r = min_cand_r / cv::norm(min_cand_r);
        max_cand_r = max_cand_r / cv::norm(max_cand_r);

        // Let's find candidators in right image
        std::vector<cv::Point3d> candidators_3d_r = unit_sphere_linspace(min_cand_r, max_cand_r, n_candidators);

        // Lets compare intensity in candidators with left intensity
        double best_intensity_diff = std::numeric_limits<double>::max(); 
        cv::Point2d best_candidator{-1, -1};
        for (int i = 0; i < candidators_3d_r.size(); i++)
        {
            cv::Point2d candidator_2d_r = r_project(candidators_3d_r[i]);

            double right_intensity = 0.0;
            for (const auto& offset: PATCH_OFFSET)
            {
                if (is_inside(candidator_2d_r + offset, r_img.image.size()))
                {
                    // cv::Point2d p = candidator_2d_r + offset;
                    right_intensity += r_img.intensity_at(candidator_2d_r + offset);
                }
            }
            right_intensity /= PATCH_OFFSET.size();

            double intensity_diff = std::abs(left_intensity - right_intensity);
            
            if (intensity_diff < best_intensity_diff && intensity_diff < 2.0)
            {
                best_intensity_diff = intensity_diff;
                best_candidator = candidator_2d_r;
            }
        }

        if (best_candidator.x != -1)
        {
            r_points[i].set_pix(best_candidator);
            r_points[i].set_idepth(1.0 / cv::norm(point_3d - l_unproject(best_candidator))); // <- 수정 필요
        }
    }

    return r_points;
}






// Util for stereo calibration
bool read_stereo_calib_result(std::string path, StereoCamera& stereo)
{
    std::ifstream ifs(path);

    Json::Value root;
    Json::Reader reader;

    bool parsingSuccessful = reader.parse(ifs, root);
    if (!parsingSuccessful) {
        std::cout << "Failed to parse configuration\n"
                  << reader.getFormattedErrorMessages();
        exit(1);
    }

    std::string l_cam_model = root["left_camera_model"].asString();
    std::string r_cam_model = root["right_camera_model"].asString();

    std::cout << "Left camera model: " << l_cam_model << std::endl;
    std::cout << "Right camera model: " << r_cam_model << std::endl;

    std::shared_ptr<CameraModel> l_cam_ptr;
    set_camera_model(l_cam_model, l_cam_ptr);
    std::shared_ptr<CameraModel> r_cam_ptr;
    set_camera_model(r_cam_model, r_cam_ptr);

    Json::Value l_params = root["left_intrinsics"];
    std::vector<double> l_params_vec;
    std::cout << "Left intrinsics: ";
    for (int i = 0; i < l_params.size(); i++)
    {
        std::cout << l_params[i].asDouble() << " ";
        l_params_vec.push_back(l_params[i].asDouble());
    }
    l_cam_ptr->set_params(l_params_vec);

    Json::Value r_params = root["right_intrinsics"];
    std::vector<double> r_params_vec;
    std::cout << "Right intrinsics: ";
    for (int i = 0; i < r_params.size(); i++)
    {
        std::cout << r_params[i].asDouble() << " ";
        r_params_vec.push_back(r_params[i].asDouble());
    }
    r_cam_ptr->set_params(r_params_vec);

    Json::Value T = root["T_from_to_c1_c2"];
    std::cout << "T_from_to_c1_c2: " << std::endl;
    for (int i = 0; i < T.size(); i++)
    {
        for (int j = 0; j < T[i].size(); j++)
        {
            std::cout << T[i][j].asDouble() << " ";
        }
        std::cout << std::endl;
    }

    cv::Mat T_mat = cv::Mat::zeros(4, 4, CV_64F);
    for (int i = 0; i < T.size(); i++)
    {
        for (int j = 0; j < T[i].size(); j++)
        {
            T_mat.at<double>(i, j) = T[i][j].asDouble();
        }
    }

    cv::Vec6d T_from_to_c1_c2;

    T_from_to_c1_c2 = convertTMatToVec6d(T_mat);

    stereo.set_left_cam(l_cam_ptr);
    stereo.set_right_cam(r_cam_ptr);
    stereo.set_T_from_to_c1_c2(T_from_to_c1_c2);
}