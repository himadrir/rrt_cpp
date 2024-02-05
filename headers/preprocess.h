#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>

namespace preprocess
{
cv::Mat read_img(std::string path);
std::vector<std::vector<int>> get_thresholded_img(const cv::Mat& image);
void add_path_to_image(const std::vector<std::array<int, 2>>& waypoints, cv::Mat& image);
};

#endif 