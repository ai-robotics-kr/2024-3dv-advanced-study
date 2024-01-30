#pragma once

#include <opencv2/opencv.hpp>
#include <json/json.h>
#include <filesystem>
#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <memory>


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

