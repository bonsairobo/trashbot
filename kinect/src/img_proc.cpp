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

vector<vector<Point2i>> find_object_regions(
    const Mat& depth, float near_thresh_depth, float far_thresh_depth)
{
    Mat depth_f32, thresh;
    depth.convertTo(depth_f32, CV_32F);
    threshold(depth_f32, thresh, far_thresh_depth, 0, THRESH_TOZERO_INV);
    threshold(thresh, thresh, near_thresh_depth, 0, THRESH_TOZERO);
    vector<vector<Point2i>> object_regions =
        find_nonzero_components<float>(thresh);
    return object_regions;
}
