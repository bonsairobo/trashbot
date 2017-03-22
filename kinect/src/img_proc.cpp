#include "img_proc.hpp"
#include <queue>

using namespace cv;
using namespace std;
using namespace openni;

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
    const VideoStream& depth_stream,
    const Mat& depth,
    const Point3f& btl,
    const Point3f& bbr,
    float near_depth)
{
    // Convert Rexarm workspace back plane to pixel coordinates.
    Point2i tl_px, br_px;
    uint16_t far_depth;
    CoordinateConverter::convertWorldToDepth(
        depth_stream, btl.x, btl.y, btl.z, &tl_px.x, &tl_px.y, &far_depth);
    CoordinateConverter::convertWorldToDepth(
        depth_stream, bbr.x, bbr.y, bbr.z, &br_px.x, &br_px.y, &far_depth);
    Mat crop = depth(
        Rect(tl_px.x, tl_px.y, br_px.x-tl_px.x, br_px.y-tl_px.y));

    Mat crop_f32;
    crop.convertTo(crop_f32, CV_32F);
    threshold(crop_f32, crop_f32, far_depth, 0, THRESH_TOZERO_INV);
    threshold(crop_f32, crop_f32, near_depth, 0, THRESH_TOZERO);
    vector<vector<Point2i>> object_regions =
        find_nonzero_components<float>(crop_f32, tl_px);
    // Reject small regions.
    const size_t min_region_size = 50;
    auto new_end = remove_if(object_regions.begin(), object_regions.end(),
        [](const vector<Point2i>& region) {
            return region.size() < min_region_size;
        }
    );
    object_regions.erase(new_end, object_regions.end());

    return object_regions;
}
