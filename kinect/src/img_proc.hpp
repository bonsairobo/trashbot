#ifndef IMG_PROC_HPP
#define IMG_PROC_HPP

#include <opencv2/opencv.hpp>
#include <vector>

cv::Mat draw_color_on_depth(const cv::Mat& color, const cv::Mat& depth);
std::vector<std::vector<cv::Point2i>> find_object_regions(
    const cv::Mat& depth, float near_thresh_depth, float far_thresh_depth);

template<typename T>
std::vector<cv::Point2i> flood_select(
    const cv::Mat& m, cv::Point2i seed, cv::Mat& visited)
{
    std::vector<cv::Point2i> component;
    std::queue<cv::Point2i> search_queue;
    search_queue.push(seed);
    visited.at<uint8_t>(seed) = 1;
    cv::Rect m_rect(cv::Point(), m.size());
    while (!search_queue.empty()) {
        cv::Point2i p = search_queue.front();
        search_queue.pop();
        component.push_back(p);

        // Spawn 4 neighbors.
        int dx = 1, dy = 0;
        for (int i = 0; i < 4; ++i) {
            cv::Point2i n(p.x+dx, p.y+dy);
            if (m_rect.contains(n) and
                !visited.at<uint8_t>(n) and
                m.at<T>(n) != 0)
            {
                search_queue.push(n);
                visited.at<uint8_t>(n) = 1;
            }
            std::swap(dx, dy);
            dx *= -1;
        }
    }
    return component;
}

template<typename T>
std::vector<std::vector<cv::Point2i>> find_nonzero_components(const cv::Mat& m)
{
    cv::Mat visited = cv::Mat::zeros(m.size(), CV_8UC1);
    std::vector<std::vector<cv::Point2i>> components;
    for (int y = 0; y < m.rows; ++y) {
        for (int x = 0; x < m.cols; ++x) {
            if (visited.at<uint8_t>(y,x) == 0 and m.at<T>(y,x) != 0) {
                components.push_back(
                    flood_select<T>(m, cv::Point2i(x,y), visited));
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
