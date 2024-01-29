#pragma once

#include <opencv2/opencv.hpp>
#include <json/json.h>
#include <filesystem>
#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>


namespace fs = std::__fs::filesystem;






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

