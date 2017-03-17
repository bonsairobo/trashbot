#ifndef IMG_PROC_HPP
#define IMG_PROC_HPP

#include <opencv2/opencv.hpp>
#include <vector>

cv::Mat draw_color_on_depth(const cv::Mat& color, const cv::Mat& depth);
std::vector<cv::Point2i> flood_select(
    const cv::Mat&, cv::Point2i seed, cv::Mat& visited);
std::vector<std::vector<cv::Point2i>> find_object_regions(
    const cv::Mat& depth, uint16_t thresh_depth);

template<typename T>
std::vector<std::vector<cv::Point2i>> find_nonzero_components(const cv::Mat& m)
{
    cv::Mat visited = cv::Mat::zeros(m.size(), CV_8UC1);
    std::vector<std::vector<cv::Point2i>> components;
    for (int y = 0; y < m.rows; ++y) {
        for (int x = 0; x < m.cols; ++x) {
            if (visited.at<uint8_t>(y,x) == 0 and m.at<T>(y,x) != 0) {
                components.push_back(
                    flood_select(m, cv::Point2i(x,y), visited));
            }
        }
    }
    return components;
}

template<typename T>
void draw_pixels(
    cv::Mat& img, std::vector<cv::Point2i> pixels, const T& color)
{
    for (auto px : pixels) {
        img.at<T>(px) = color;
    }
}

#endif // IMG_PROC_HPP
