#include "camera/cameras.hpp"
#include "frame/frame.hpp"
#include "board/chessboard.hpp"
#include "util.hpp"

#include <vector>
#include <memory>
#include <iostream>
#include <cmath>

#include <thread>
#include <mutex>



struct DoubleSphereStereoCalib
{
    cv::Point2d observed_;
    cv::Point3d point_;

    DoubleSphereStereoCalib(
        const cv::Point2d& observed, 
        const cv::Point3d& point)
        : observed_(observed), point_(point) {}

    template <typename T>
    bool operator() (
        const T* const intrinsic,
        const T* const T_w_to_c1, 
        const T* const T_c1_to_c2,
        T* residuals) const
    {
        T fx = intrinsic[0];
        T fy = intrinsic[1];
        T cx = intrinsic[2];
        T cy = intrinsic[3];
        T xi = intrinsic[4];
        T alpha = intrinsic[5];

        T l_P[3], r_P[3];
        T point[3] = {T(point_.x), T(point_.y), T(point_.z)};

        // To left coordinate
        ceres::AngleAxisRotatePoint(T_w_to_c1, point, l_P);

        l_P[0] += T_w_to_c1[3];
        l_P[1] += T_w_to_c1[4];
        l_P[2] += T_w_to_c1[5];

        // To right coordinate
        ceres::AngleAxisRotatePoint(T_c1_to_c2, l_P, r_P);

        r_P[0] += T_c1_to_c2[3];
        r_P[1] += T_c1_to_c2[4];
        r_P[2] += T_c1_to_c2[5];

        T x = r_P[0];
        T y = r_P[1];
        T z = r_P[2];
        T one = T(1);

        T d1 = ceres::sqrt(x * x + y * y + z * z);
        T d2 = ceres::sqrt(x * x + y * y + (xi * d1 + z) * (xi * d1 + z));

        T tmp = alpha * d2 + (one - alpha) * (xi * d1 + z);

        T u = fx * x / tmp + cx;
        T v = fy * y / tmp + cy;

        residuals[0] = u - T(observed_.x);
        residuals[1] = v - T(observed_.y);

        return true;
    }

    static ceres::CostFunction* create(const cv::Point2d& r_observed, const cv::Point3d& point)
    {
        return (new ceres::AutoDiffCostFunction<DoubleSphereStereoCalib, 2, 6, 6, 6>(new DoubleSphereStereoCalib(r_observed, point)));
    }

};











/// @brief Calibrate camera as mono and save it into cam and T_w2c.
/// @param cam_cofig: input config obj
/// @param frames: input config obj
/// @param cam: specified camera model
/// @param T_w2c: output vector of transformation matrix from world to camera
void monocalib(const Cfg& cfg,
               std::shared_ptr<CameraModel>& cam, 
               std::vector<Frame>& frames,
               ChessBoard& board) noexcept
{
    // init camera intrinsic parameters
    double f, cx, cy;

    cv::Mat tmp = frames[0].image;
    
    int width = tmp.cols;
    int height = tmp.rows;

    f = static_cast<double>(width / 4);
    cx = static_cast<double>((width - 1.0) / 2.0);
    cy = static_cast<double>((height - 1.0) / 2.0);    
    cam->init_params(f, cx, cy);
    // Find initial pose
    for (auto& frame: frames)
        if (!board.find_initial_pose(*cam, frame))
            std::cout << "Failed to find good pose for frame: " << frame.id << std::endl;

    // Build residual block
    double huber_loss_threshold = 1.0;
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(huber_loss_threshold);

    std::vector<double> camera_params = cam->get_params();
    double* p_camera_params = camera_params.data();

    double scale = 1.0;

    ceres::Problem problem;
    std::vector<cv::Vec6d> frame_poses(frames.size());

    // Add residual blocks
    for (size_t i = 0; i < frames.size(); i++) {
        frame_poses[i] = frames[i].get_transform();

        for (size_t j = 0; j < frames[i].get_corner_num(); j++) {
            cv::Point2d object_pixel = frames[i].get_corner(j);
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
                return ;
            }

            double* p_frame_pose = (double*) (&frame_poses[i]);

            problem.AddResidualBlock(cost_function, loss_function, p_camera_params, p_frame_pose);
        }
    }

    // std::cout << "Let's calibrate camera!!" << std::endl;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.num_threads = 8;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 500;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Set result
    cam->set_params(camera_params);

    for (size_t i = 0; i < frames.size(); i++)
        frames[i].set_transform(frame_poses[i]);

    std::cout << "Camera calibration done!" << std::endl;
    std::cout << summary.BriefReport() << std::endl;
}


using std::sqrt;


int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Usage: ./stereo_calibration </path/to/cam1.json> </path/to/cam2.json>";
        return 0;
    }

    std::string cam1_config_file = argv[1];
    std::string cam2_config_file = argv[2];

    std::cout << "Loading config file: " << cam1_config_file << std::endl;

    // load config file
    Cfg cam1_cfg = read_json(cam1_config_file);
    Cfg cam2_cfg = read_json(cam2_config_file);

    cam1_cfg.show();
    cam2_cfg.show();

    std::shared_ptr<CameraModel> cam1; set_camera_model(cam1_cfg.camera_model, cam1);
    std::shared_ptr<CameraModel> cam2; set_camera_model(cam2_cfg.camera_model, cam2);


    // init chessboard
    ChessBoard board(cam1_cfg.board_pattern, cam1_cfg.board_size);

    // init frames
    std::vector<std::string> path_to_image1 = read_images(cam1_cfg.data_path);
    std::vector<std::string> path_to_image2 = read_images(cam2_cfg.data_path);

    std::vector<Frame> frames1;
    std::vector<Frame> frames2;

    for (int i = 0; i < path_to_image1.size(); i++)
    {
        Frame frame1(i, path_to_image1[i]);
        if (board.find_corners(frame1))
            frames1.push_back(frame1);
    }

    for (int i = 0; i < path_to_image2.size(); i++)
    {
        Frame frame2(i, path_to_image2[i]);
        if (board.find_corners(frame2))
            frames2.push_back(frame2);
    }

    if (frames1.size() != frames2.size())
    {
        std::cout << "Number of frames are not same!" << std::endl;
        return 0;
    }

    ///////// After monocular calibration, optimize stereo camera. /////////
    std::thread t1(monocalib, std::ref(cam1_cfg), std::ref(cam1), std::ref(frames1), std::ref(board));
    std::thread t2(monocalib, std::ref(cam2_cfg), std::ref(cam2), std::ref(frames2), std::ref(board));

    t1.join();
    t2.join();

    ///////// Don't monocular calibration, just optimize stereo camera. /////////
    // double f, cx, cy;
    // double width = frames1[0].width;
    // double height = frames1[0].height;
    // f = width / 4;
    // cx = (width - 1.0) / 2;
    // cy = (height - 1.0) / 2;

    // cam1->init_params(f, cx, cy);
    // cam2->init_params(f, cx, cy);

    // for (size_t i = 0; i < frames1.size(); i++)
    // {
    //     if (!board.find_initial_pose(*cam1, frames1[i]))
    //         std::cout << "Failed to find good pose for frame left: " << frames1[i].id << std::endl;
    //     if (!board.find_initial_pose(*cam2, frames2[i]))
    //         std::cout << "Failed to find good pose for frame right: " << frames2[i].id << std::endl;
    // }


    std::cout << "Start Stereo Calibration!" << std::endl;

    cv::Vec6d T_c1_to_c2 = cv::Vec6d(0, 0, 0, -0.064, 0, 0);
    double* p_T_c1_to_c2 = (double*) (&T_c1_to_c2);
    
    std::cout << "Before calib -> T_c1_to_c2: " << T_c1_to_c2 << std::endl;

    // Do stereo calibration with ceres
    std::vector<double> l_intrinsic = cam1->get_params();
    double* p_l_intrinsic = l_intrinsic.data();

    std::vector<double> r_intrinsic = cam2->get_params();
    double* p_r_intrinsic = r_intrinsic.data();

    std::vector<cv::Vec6d> T_w_to_c1(frames1.size());

    std::cout << "Start optimization with ceres-solver" << std::endl;
    ceres::Problem problem;
    double huber_loss_threshold = 1.0;
    ceres::LossFunction* loss_function = new ceres::HuberLoss(huber_loss_threshold);

    for (size_t i = 0; i < frames1.size(); i++)
    {
        T_w_to_c1[i] = frames1[i].get_transform();

        for (size_t j = 0; j < frames1[i].get_corner_num(); j++)
        {
            cv::Point2d l_observed = frames1[i].get_corner(j);
            cv::Point2d r_observed = frames2[i].get_corner(j);
            cv::Point3d point = board.get_board_point(j);

            ceres::CostFunction* l_cost_function = 
                DoubleSphere::ReprojectionError::create(
                    l_observed, 
                    point);

            ceres::CostFunction* r_cost_function = 
                DoubleSphereStereoCalib::create(
                    r_observed, 
                    point);

            double* p_T_w_to_c1 = (double*) (&T_w_to_c1[i]);

            // left part
            problem.AddResidualBlock(
                l_cost_function, 
                loss_function, 
                p_l_intrinsic,
                p_T_w_to_c1);
            
            // right part
            problem.AddResidualBlock(
                r_cost_function,
                loss_function,
                p_r_intrinsic,
                p_T_w_to_c1,
                p_T_c1_to_c2
            );
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.num_threads = 8;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 500;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;

    std::cout << "Before calibration: " << std::endl;
    std::cout << "Left camera intrinsic: ";
    for (auto& param: cam1->get_params())
        std::cout << param << " ";
    std::cout << std::endl;

    std::cout << "Right camera intrinsic: ";
    for (auto& param: cam2->get_params())
        std::cout << param << " ";
    std::cout << std::endl;

    std::cout << "After calibration: " << std::endl;
    std::cout << "Left camera intrinsic: ";
    for (auto& param: l_intrinsic)
        std::cout << param << " ";
    std::cout << std::endl;

    std::cout << "Right camera intrinsic: ";
    for (auto& param: r_intrinsic)
        std::cout << param << " ";
    std::cout << std::endl;

    cv::Mat T_c1_to_c2_mat = convertVec6dToTransformMatrix(T_c1_to_c2);

    // Save as json
    std::string output_file = "../data/stereo_calibration_result.json";

    save_stereo_calib_as_json(
        output_file,
        cam1_cfg.camera_model,
        cam2_cfg.camera_model,
        l_intrinsic,
        r_intrinsic,
        T_c1_to_c2
    );
}