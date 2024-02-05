#include "preprocess.h"
#include <opencv2/opencv.hpp>

namespace preprocess
{
cv::Mat read_img(std::string path)
{   
    cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
    return image;
}

std::vector<std::vector<int>> get_thresholded_img(const cv::Mat& image) 
{
    int rows = image.rows;
    int cols = image.cols;
    std::vector<std::vector<int>> thresholded_data(rows, std::vector<int>(cols, 0));

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j) 
        {
            uchar pixel_value = image.at<uchar>(i, j);
            thresholded_data[i][j] = (pixel_value > 0) ? 1 : 0;  // 1 for free, 0 for occupied
        }
    }

    return thresholded_data;
}

void add_path_to_image(const std::vector<std::array<int, 2>>& waypoints, cv::Mat& image)
{
    for(size_t i=0; i<waypoints.size()-1; i++)
    {
        cv::Point p1(waypoints[i][0], waypoints[i][1]);
        cv::Point p2(waypoints[i+1][0], waypoints[i+1][1]);
        cv::line(image, p1, p2, cv::Scalar(0), 1, cv::LINE_AA);
    }
}
};