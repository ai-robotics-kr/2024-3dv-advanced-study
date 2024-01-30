#include "camera/camera_model_template.hpp"
#include "camera/brown_conrady.hpp"
#include "camera/kannala_brandt.hpp"
#include "camera/single_sphere.hpp"
#include "camera/double_sphere.hpp"
#include "camera/triple_sphere.hpp"
#include "frame/frame.hpp"
#include "board/chessboard.hpp"
#include "util.hpp"

#include <vector>
#include <memory>
#include <iostream>
#include <cmath>




using std::sqrt;




int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: ./calibration <path_to_config_file>" << std::endl;
        return 0;
    }

    std::string config_file = argv[1];

    std::cout << "Loading config file: " << config_file << std::endl;

    Cfg cfg = read_json(config_file);

    cfg.show();

    std::vector<std::string> path_to_image = read_images(cfg.data_path);
    std::vector<Frame> frames;

    for (int i = 0; i < path_to_image.size(); i++)
    {
        Frame frame(i, path_to_image[i]);
        frames.push_back(frame);
    }

    ChessBoard board(cfg.board_pattern, cfg.board_size);

    // board.show_board_info();

    std::unique_ptr<CameraModel> cam;

    if (cfg.camera_model == "BrownConrady")
    {
        cam = std::make_unique<BrownConrady>();
    }
    else if (cfg.camera_model == "KannalaBrandt")
    {
        cam = std::make_unique<KannalaBrandt>();
    }
    else if (cfg.camera_model == "SingleSphere")
    {
        cam = std::make_unique<SingleSphere>();
    }
    else if (cfg.camera_model == "DoubleSphere")
    {
        cam = std::make_unique<DoubleSphere>();
    }
    else if (cfg.camera_model == "TripleSphere")
    {
        cam = std::make_unique<TripleSphere>();
    }
    else
    {
        std::cout << "Invalid camera model: " << cfg.camera_model << std::endl;
        return -1;
    }


    std::vector<std::vector<cv::Point2d>> corners(frames.size());
    std::vector<cv::Vec6d> camera_poses(frames.size());

    std::vector<int> good_frames_id;

    for (int i = 0; i < frames.size(); i++)
    {
        if (board.find_corners(frames[i]))
            good_frames_id.push_back(i);
    }
    
    std::vector<Frame> good_frames;
    for (const auto& id : good_frames_id)
        good_frames.push_back(frames[id]);

    std::cout << "Number of good frames: " << good_frames.size() << std::endl;

    // Init camera params
    double f, cx, cy;

    cv::Mat tmp = good_frames[0].image;
    
    int width = tmp.cols;
    int height = tmp.rows;

    f = static_cast<double>(width / 4);
    cx = static_cast<double>((width - 1.0) / 2.0);
    cy = static_cast<double>((height - 1.0) / 2.0);
    
    cam->init_params(f, cx, cy);

    // Find initial pose
    for (auto& frame: good_frames)
        if (!board.find_initial_pose(*cam, frame))
            std::cout << "Failed to find good pose for frame: " << frame.id << std::endl;

    
    // Optimize with ceres
    std::cout << "Optimzing with ceres..." << std::endl;
    ceres::Problem problem;

    std::vector<double> camera_params = cam->get_params();
    double* p_camera_params = camera_params.data();

    std::cout << "Length of initial camera parameters: " << camera_params.size() << std::endl;


    std::vector<cv::Vec6d> frame_poses(good_frames.size());

    double huber_loss_threshold = 1.0;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(huber_loss_threshold);


    for (size_t i = 0; i < good_frames.size(); i++) {
        frame_poses[i] = good_frames[i].get_transform();

        for (size_t j = 0; j < good_frames[i].get_corner_num(); j++) {
            cv::Point2d object_pixel = good_frames[i].get_corner(j);
            cv::Point3d point = board.get_board_point(j);

            ceres::CostFunction* cost_function;

            if (cfg.camera_model == "BrownConrady")
                cost_function = BrownConrady::ReprojectionError::create(object_pixel, point);
            else if (cfg.camera_model == "KannalaBrandt")
                cost_function = KannalaBrandt::ReprojectionError::create(object_pixel, point);
            else if (cfg.camera_model == "SingleSphere")
                cost_function = SingleSphere::ReprojectionError::create(object_pixel, point);
            else if (cfg.camera_model == "DoubleSphere")
                cost_function = DoubleSphere::ReprojectionError::create(object_pixel, point);
            else if (cfg.camera_model == "TripleSphere")
                cost_function = TripleSphere::ReprojectionError::create(object_pixel, point);
            else
            {
                std::cout << "Invalid camera model: " << cfg.camera_model << std::endl;
                return -1;
            }

            double* p_frame_pose = (double*) (&frame_poses[i]);

            problem.AddResidualBlock(cost_function, loss_function, p_camera_params, p_frame_pose);
        }
    }


    if (cfg.camera_model == "TripleSphere") {
        problem.SetParameterUpperBound(p_camera_params, 4, 1.000);
        problem.SetParameterUpperBound(p_camera_params, 5, 1.000);
        problem.SetParameterUpperBound(p_camera_params, 6, 1.000);

        problem.SetParameterLowerBound(p_camera_params, 4, 0.0001);
        problem.SetParameterLowerBound(p_camera_params, 5, 0.0001);
        problem.SetParameterLowerBound(p_camera_params, 6, 0.0001);
    }

    std::cout << "Let's calibrate camera!!" << std::endl;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.num_threads = 8;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 500;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << "Initial camera parameters: " << std::endl;
    std::cout << "fx: " << f << ", fy: " << f << ", cx: " << cx << ", cy: " << cy << std::endl;
    std::cout << "Final camera parameters: " << std::endl;
    for (const auto& param : camera_params)
        std::cout << param << " ";
    std::cout << std::endl;

    // Set camera params and show them
    cam->set_params(camera_params);    

    // Set frame poses
    for (size_t i = 0; i < good_frames.size(); i++)
        good_frames[i].set_transform(frame_poses[i]);
    
    // Show reprojection error
    double total_error = 0.0;
    for (size_t i = 0; i < good_frames.size(); i++) {
        for (size_t j = 0; j < good_frames[i].get_corner_num(); j++) {
            cv::Point2d object_pixel = good_frames[i].get_corner(j);
            cv::Point3d point = board.get_board_point(j);

            // Rotate point into camera coordinate
            cv::Vec6d pose = good_frames[i].get_transform();
            cv::Mat R;
            cv::Rodrigues(cv::Vec3d(pose[0], pose[1], pose[2]), R);
            cv::Mat t = (cv::Mat_<double>(3, 1) << pose[3], pose[4], pose[5]);
            cv::Mat point_mat = (cv::Mat_<double>(3, 1) << point.x, point.y, point.z);
            cv::Mat rotated_point_mat = R * point_mat + t;

            cv::Point3d rotated_point(rotated_point_mat.at<double>(0, 0), 
                                      rotated_point_mat.at<double>(1, 0), 
                                      rotated_point_mat.at<double>(2, 0));

            cv::Point2d reprojected_pixel = cam->project(rotated_point);

            double error_x = object_pixel.x - reprojected_pixel.x;
            double error_y = object_pixel.y - reprojected_pixel.y;

            double error = sqrt(error_x * error_x + error_y * error_y);

            total_error += error;
        }
    }
    
    total_error /= (good_frames.size() * good_frames[0].get_corner_num());

    std::cout << "Mean reprojection error: " << total_error << std::endl;

    if (save_as_json(cfg.output_path, cfg.output_name, cfg.camera_model, cam->get_params(),
                     good_frames.size(),
                     total_error))
        std::cout << "Save as json file!\n";

    return 0;
}