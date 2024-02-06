#pragma once

#include "camera/camera_model_template.hpp"
#include "camera/brown_conrady.hpp"
#include "camera/kannala_brandt.hpp"
#include "camera/single_sphere.hpp"
#include "camera/double_sphere.hpp"
#include "camera/triple_sphere.hpp"

#include <memory>
#include <string>



void set_camera_model(std::string camera_model, std::shared_ptr<CameraModel>& cam)
{
    if (camera_model == "BrownConrady")
    {
        cam = std::make_shared<BrownConrady>();
    }
    else if (camera_model == "KannalaBrandt")
    {
        cam = std::make_shared<KannalaBrandt>();
    }
    else if (camera_model == "SingleSphere")
    {
        cam = std::make_shared<SingleSphere>();
    }
    else if (camera_model == "DoubleSphere")
    {
        cam = std::make_shared<DoubleSphere>();
    }
    else if (camera_model == "TripleSphere")
    {
        cam = std::make_shared<TripleSphere>();
    }
    else
    {
        std::cout << "Invalid camera model: " << camera_model << std::endl;
    }    
}