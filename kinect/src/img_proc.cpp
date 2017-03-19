#include "img_proc.hpp"
#include <queue>

using namespace cv;
using namespace std;

Mat draw_color_on_depth(const Mat& color, const Mat& depth) {
    Mat out = Mat::zeros(color.size(), CV_8UC3);
    for (int y = 0; y < out.rows; ++y) {
        for (int x = 0; x < out.cols; ++x) {
            if (depth.at<uint16_t>(y,x) != 0) {
                out.at<Vec3b>(y,x) = color.at<Vec3b>(y,x);
            }
        }
    }
    return out;
}

vector<Point2i> flood_select(const Mat& m, Point2i seed, Mat& visited) {
    vector<Point2i> component;
    queue<Point2i> search_queue;
    search_queue.push(seed);
    visited.at<uint8_t>(seed) = 1;
    Rect m_rect(Point(), m.size());
    while (!search_queue.empty()) {
        Point2i p = search_queue.front();
        search_queue.pop();
        component.push_back(p);

        // Spawn 4 neighbors.
        int dx = 1, dy = 0;
        for (int i = 0; i < 4; ++i) {
            Point2i n(p.x+dx, p.y+dy);
            if (m_rect.contains(n) and
                !visited.at<uint8_t>(n) and
                m.at<uint16_t>(n) != 0)
            {
                search_queue.push(n);
                visited.at<uint8_t>(n) = 1;
            }
            swap(dx, dy);
            dx *= -1;
        }
    }
    return component;
}

vector<vector<Point2i>> find_object_regions(
    const Mat& depth, uint16_t thresh_depth)
{
    Mat near_depth;
    threshold(-depth, near_depth, -thresh_depth, 0, THRESH_TOZERO);
    vector<vector<Point2i>> object_regions =
        find_nonzero_components<uint16_t>(near_depth);
    return object_regions;
}
