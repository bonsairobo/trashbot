#include "img_proc.hpp"
#include <pcl/features/integral_image_normal.h>
#include <queue>
#include <cassert>

using namespace cv;
using namespace std;
using namespace openni;
using namespace pcl;

PointCloud<PointXYZ>::Ptr zero_cloud(int width, int height) {
    PointCloud<PointXYZ>::Ptr pc(new PointCloud<PointXYZ>);
    pc->width = width;
    pc->height = height;
    pc->resize(width * height);
    pc->is_dense = true;
    return pc;
}

PlaneInfo remove_planes(PointCloud<PointXYZ>::ConstPtr pc) {
    PlaneInfo info;

    // Estimate normals.
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
    IntegralImageNormalEstimation<PointXYZ, Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(pc);
    ne.compute(*normals);
    info.normals = normals;

    // Do plane segmentation.


    return info;
}

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

vector<vector<Point2i>> get_workspace_pixels(
    const VideoStream& depth_stream,
    const Mat& depth,
    const Point3f& ftl,
    const Point3f& bbr,
    Rect* roi_out)
{
    // Convert Rexarm workspace vertices to pixel coordinates.
    const vector<Point3f> workspc_corners = {
        ftl,
        bbr,
        Point3f(bbr.x, ftl.y, ftl.z),
        Point3f(ftl.x, bbr.y, ftl.z),
        Point3f(ftl.x, ftl.y, bbr.z),
        Point3f(bbr.x, bbr.y, ftl.z),
        Point3f(ftl.x, bbr.y, bbr.z),
        Point3f(bbr.x, ftl.y, bbr.z)
    };
    int min_x, max_x, min_y, max_y;
    min_x = min_y = numeric_limits<int>::max();
    max_x = max_y = 0;
    for (const auto& c : workspc_corners) {
        Point2i px;
        uint16_t z;
        CoordinateConverter::convertWorldToDepth(
            depth_stream, c.x, c.y, c.z, &px.x, &px.y, &z);
        if (px.x < min_x)
            min_x = px.x;
        if (px.x > max_x)
            max_x = px.x;
        if (px.y < min_y)
            min_y = px.y;
        if (px.y > max_y)
            max_y = px.y;
    }
    Rect roi(0, 0, depth.cols, depth.rows);
    if (min_x > 0 and min_x < depth.cols)
        roi.x = min_x;
    if (max_x > 0 and max_x < depth.cols)
        roi.width = max_x - min_x;
    if (min_y > 0 and min_y < depth.rows)
        roi.y = min_y;
    if (max_y > 0 and max_y < depth.rows)
        roi.height = max_y - min_y;

    // Select ROI from depth image.
    Mat crop = depth(roi);

    // Do depth thresholding.
    Mat thresh;
    crop.convertTo(thresh, CV_32F);
    threshold(thresh, thresh, bbr.z, 0, THRESH_TOZERO_INV);
    threshold(thresh, thresh, ftl.z, 0, THRESH_TOZERO);

    // Extract regions as sets of pixels.
    vector<vector<Point2i>> object_regions =
        find_nonzero_components<float>(thresh);

    // Reject small regions.
    const size_t min_region_size = 50;
    auto new_end = remove_if(object_regions.begin(), object_regions.end(),
        [](const vector<Point2i>& region) {
            return region.size() < min_region_size;
        }
    );
    object_regions.erase(new_end, object_regions.end());

    if (roi_out != nullptr)
        *roi_out = roi;

    return object_regions;
}
