#pragma once

#include "camera/cameras.hpp"
// #include "depth_estimator/depth_estimator.hpp"

#include <opencv2/opencv.hpp>
#include <json/json.h>
#include <filesystem>
#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <memory>
#include <cmath>
#include <cassert>




// macOS에서는 std::filesystem이 아직 실험적인 기능이므로
#if defined(__APPLE__) || defined(__MACOSX)
    namespace fs = std::__fs::filesystem;
#else
    namespace fs = std::filesystem;
#endif



struct Cfg
{
    cv::Size board_pattern;
    std::array<double, 2> board_size;
    std::string camera_model;
    std::string data_path;
    std::string output_path;
    std::string output_name;

    void show()
    {
        std::cout << "Board pattern: " << board_pattern << std::endl;
        std::cout << "Board size: " << board_size[0] << "X" << board_size[1] << std::endl;
        std::cout << "Camera model: " << camera_model << std::endl;
        std::cout << "Data path: " << data_path << std::endl;
        std::cout << "Output path: " << output_path << std::endl;
        std::cout << "Output name: " << output_name << std::endl;
    }
};


Cfg read_json(const std::string& json_file)
{
    std::ifstream ifs(json_file);

    Json::Value root;
    Json::Reader reader;

    bool parsingSuccessful = reader.parse(ifs, root);
    if (!parsingSuccessful) {
        std::cout << "Failed to parse configuration\n"
                  << reader.getFormattedErrorMessages();
        exit(1);
    }

    Cfg cfg;

    Json::Value pattern = root["board_pattern"];
    cfg.board_pattern = cv::Size(pattern[0].asInt(), pattern[1].asInt());
    Json::Value board_size = root["board_size"];
    cfg.board_size = {board_size[0].asDouble(), board_size[1].asDouble()};
    cfg.camera_model = root["camera_model"].asString();
    cfg.data_path = root["data_path"].asString();
    cfg.output_path = root["output_path"].asString();
    cfg.output_name = root["output_name"].asString();

    return cfg;
}


bool save_as_json(std::string output_path,
                  std::string output_name,
                  std::string cam_model, 
                  const std::vector<double>& parameters,
                  const int& number_of_images,
                  const double& avg_res)
{
    Json::Value root;
    root["camera_model"] = cam_model;
    
    Json::Value paramsArray(Json::arrayValue);
    for (double param : parameters) {
        paramsArray.append(param);
    }
    root["parameters"] = paramsArray;

    root["number_of_used_images"] = number_of_images;
    root["avg_residual"] = avg_res;

    // 파일 저장
    std::ofstream file(output_path + output_name);
    Json::StreamWriterBuilder builder;
    builder["commentStyle"] = "None";
    builder["indentation"] = "    ";  // 적절한 들여쓰기 설정
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(root, &file);

    return true;
}


struct CalibResult
{
    std::string camera_model;
    std::vector<double> parameters;
    int number_of_images;
    double avg_residual;

    void show()
    {
        std::cout << "Camera model: " << camera_model << std::endl;
        std::cout << "Parameters: ";
        for (double param : parameters) {
            std::cout << param << " ";
        }
        std::cout << std::endl;
        std::cout << "Number of images: " << number_of_images << std::endl;
        std::cout << "Average residual: " << avg_residual << std::endl;
    }
};

CalibResult read_calib_result(std::string file)
{
    std::ifstream ifs(file);

    Json::Value root;
    Json::Reader reader;

    bool parsingSuccessful = reader.parse(ifs, root);
    if (!parsingSuccessful) {
        std::cout << "Failed to parse configuration\n"
                  << reader.getFormattedErrorMessages();
        exit(1);
    }

    CalibResult result;

    result.camera_model = root["camera_model"].asString();

    Json::Value params = root["parameters"];
    for (int i = 0; i < params.size(); i++)
    {
        result.parameters.push_back(params[i].asDouble());
    }

    result.number_of_images = root["number_of_used_images"].asInt();
    result.avg_residual = root["avg_residual"].asDouble();

    return result;
}


std::vector<std::string> read_images(const std::string& data_path, std::string extension = ".png")
{
    std::vector<std::string> images;

    if (fs::exists(data_path) && fs::is_directory(data_path)) 
    {
        std::cout << "Data path exists." << std::endl;
        for (const auto& entry: fs::directory_iterator(data_path)) 
        {
            auto filename = entry.path().filename().string();
            if (entry.path().extension() == extension)
            {
                images.emplace_back(entry.path().string());
            }
        }
        std::cout << "Found " << images.size() << " images." << std::endl;
    } else {
        std::cout << "Data path does not exist." << std::endl;
    }

    if (images.empty()) {
        std::cout << "No images found." << std::endl;
    }

    return images;
}


cv::Mat convertVec6dToTransformMatrix(const cv::Vec6d& vec6d) {
    // 회전 벡터 (rvec)와 변환 벡터 (tvec) 추출
    cv::Vec3d rvec(vec6d[0], vec6d[1], vec6d[2]);
    cv::Vec3d tvec(vec6d[3], vec6d[4], vec6d[5]);

    // 회전 벡터를 회전 행렬로 변환
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // 4x4 변환 행렬 생성
    cv::Mat T = cv::Mat::eye(4, 4, R.type());
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    for (int i = 0; i < 3; ++i) {
        T.at<double>(i, 3) = tvec[i];
    }

    return T;
}


cv::Vec6d convertTMatToVec6d(const cv::Mat& mat) {
    // 회전 벡터를 회전 행렬로 변환
    cv::Mat R = mat(cv::Rect(0, 0, 3, 3));
    cv::Vec3d rvec;
    cv::Rodrigues(R, rvec);

    // 변환 벡터 추출
    cv::Vec3d tvec;
    for (int i = 0; i < 3; ++i) {
        tvec[i] = mat.at<double>(i, 3);
    }

    cv::Vec6d T;
    T[0] = rvec[0];
    T[1] = rvec[1];
    T[2] = rvec[2];
    T[3] = tvec[0];
    T[4] = tvec[1];
    T[5] = tvec[2];

    return T;
}



std::vector<double> linspace(double start, double end, int num)
{
    std::vector<double> linspaced;
    if (num == 0) { return linspaced; }
    if (num == 1) {
        linspaced.push_back(start);
        return linspaced;
    }
    double delta = (end - start) / (num - 1);
    for(int i = 0; i < num - 1; ++i) {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end);
    return linspaced;
}



std::vector<cv::Point3d> unit_sphere_linspace(
    const cv::Point3d& start, 
    const cv::Point3d& end, 
    int num)
{
    std::vector<cv::Point3d> linspaced;

    if (num == 0) { return linspaced; }
    if (num == 1) {
        linspaced.push_back(start);
        return linspaced;
    }

    double dx = (end.x - start.x) / num;
    double dy = (end.y - start.y) / num;
    double dz = (end.z - start.z) / num;

    for (int i = 0; i < num; i++)
    {
        cv::Point3d point;
        point.x = start.x + dx * i;
        point.y = start.y + dy * i;
        point.z = start.z + dz * i;
        point /= std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
        linspaced.push_back(point);
    }

    return linspaced;
}





bool save_stereo_calib_as_json(
    std::string output_name,
    std::string l_cam_model, 
    std::string r_cam_model, 
    const std::vector<double>& l_parameters,
    const std::vector<double>& r_parameters,
    const cv::Vec6d& T_from_to_c1_c2)
{
    Json::Value root;

    root["left_camera_model"] = l_cam_model;
    Json::Value l_paramsArray(Json::arrayValue);
    for (double param : l_parameters)
        l_paramsArray.append(param);
    
    root["left_intrinsics"] = l_paramsArray;

    root["right_camera_model"] = r_cam_model;
    Json::Value r_paramsArray(Json::arrayValue);
    for (double param : r_parameters)
        r_paramsArray.append(param);
    
    root["right_intrinsics"] = r_paramsArray;


    cv::Mat transform = convertVec6dToTransformMatrix(T_from_to_c1_c2);
    Json::Value T(Json::arrayValue);
    for (int i = 0; i < transform.rows; i++)
    {
        Json::Value row(Json::arrayValue);
        for (int j = 0; j < transform.cols; j++)
        {
            row.append(transform.at<double>(i, j));
        }
        T.append(row);
    }
    root["T_from_to_c1_c2"] = T;

    // 파일 저장
    std::ofstream file(output_name);
    Json::StreamWriterBuilder builder;
    builder["commentStyle"] = "None";
    builder["indentation"] = "    ";  // 적절한 들여쓰기 설정
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(root, &file);

    return true;
}