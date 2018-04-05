//
// Created by zhanghm on 17-12-27.
//

#ifndef DEPTH_IMAGE_GENERATOR_UTILS_H
#define DEPTH_IMAGE_GENERATOR_UTILS_H

#include <string>

#include "./cloud.h"
#include <opencv2/opencv.hpp>

namespace dc = depth_clustering;



dc::Cloud::Ptr CloudFromFile(const std::string &file_name,const std::string& file_type,
                             const dc::ProjectionParams &proj_params);

#endif //DEPTH_IMAGE_GENERATOR_UTILS_H
