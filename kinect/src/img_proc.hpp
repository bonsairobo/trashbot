#ifndef IMG_PROC_HPP
#define IMG_PROC_HPP

#include <opencv2/opencv.hpp>
#include <vector>

cv::Mat draw_color_on_depth(const cv::Mat& color, const cv::Mat& depth);
std::vector<cv::Point2i> flood_select(
    const cv::Mat&, cv::Point2i seed, cv::Mat& visited);
std::vector<std::vector<cv::Point2i>> find_nonzero_components(const cv::Mat&);
std::vector<std::vector<cv::Point2i>> find_object_regions(
    const cv::Mat& depth, uint16_t thresh_depth);

template<typename T>
void draw_pixels(
    cv::Mat& img, std::vector<cv::Point2i> pixels, const T& color)
{
    for (auto px : pixels) {
        img.at<T>(px) = color;
    }
}

#endif // IMG_PROC_HPP
