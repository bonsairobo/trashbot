#include "img_proc.hpp"
#include <pcl/features/integral_image_normal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <queue>
#include <cassert>

using namespace cv;
using namespace std;
using namespace openni;
using namespace pcl;

static const size_t MIN_REGION_SIZE = 50;

ObjectInfo get_workspace_objects(
    const VideoStream& depth_stream, const Mat& depth_mat)
{
    // Do workspace pixel culling.
    Point3f ftl(-300.0, 300.0, 800.0);
    Point3f bbr(300.0, -300.0, 1200.0);
    Rect roi;
    vector<vector<Point2i>> workspc_px =
        get_workspace_pixels(depth_stream, depth_mat, ftl, bbr, &roi);
    Point2i tl_px = Point2i(roi.x, roi.y);

    // Create a point cloud of ROI regions.
    PointCloud<PointXYZ>::Ptr pc = zero_cloud(roi.width, roi.height);
    for (const auto& region : workspc_px) {
        for (const auto& px : region) {
            PointXYZ& pt = pc->at(px.x, px.y);
            CoordinateConverter::convertDepthToWorld(
                depth_stream,
                px.x+roi.x, px.y+roi.y,
                depth_mat.at<uint16_t>(px + tl_px),
                &pt.x, &pt.y, &pt.z);
            pt.z *= -1.0;
        }
    }

    // Remove planes.
    vector<int> idx_px_map;
    PointCloud<PointXYZ>::Ptr object_pc = remove_planes(pc, &idx_px_map);

    // Cluster remaining points.
    vector<PointIndices> cluster_idx;
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
    tree->setInputCloud(object_pc);
    EuclideanClusterExtraction<PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(MIN_REGION_SIZE);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(object_pc);
    ec.extract(cluster_idx);

    // Convert clusters back into pixel coordinates.
    vector<vector<Point2i>> object_px;
    for (const auto& cluster : cluster_idx) {
        vector<Point2i> px_coords;
        for (const auto& i : cluster.indices) {
            px_coords.push_back(
                tl_px +
                Point2i(idx_px_map[i] % roi.width, idx_px_map[i] / roi.width));
        }
        object_px.push_back(px_coords);
    }

    return { pc, object_px, roi };
}

PointCloud<PointXYZ>::Ptr zero_cloud(int width, int height) {
    PointCloud<PointXYZ>::Ptr pc(new PointCloud<PointXYZ>);
    pc->width = width;
    pc->height = height;
    pc->resize(width * height);
    pc->is_dense = true;
    return pc;
}

PointCloud<Normal>::Ptr estimate_normals(PointCloud<PointXYZ>::ConstPtr pc) {
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
    IntegralImageNormalEstimation<PointXYZ, Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(pc);
    ne.compute(*normals);
    return normals;
}

PointCloud<PointXYZ>::Ptr remove_planes(
    PointCloud<PointXYZ>::ConstPtr pc,
    vector<int> *indices_out)
{
    // Do plane segmentation.
    PointCloud<pcl::PointXYZ>::Ptr filtered = pc->makeShared();
    vector<int> keep_idx;
    bool first_time = true;
    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    PointIndices::Ptr inliers(new PointIndices);
    SACSegmentation<PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setMaxIterations(500);
    seg.setDistanceThreshold(5);
    ExtractIndices<PointXYZ> extract;
    int nr_points = (int) filtered->points.size();
    while (true) {
        // Segment the largest planar component from the remaining cloud.
        seg.setInputCloud(filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            cerr << "Could not estimate a planar model for the given dataset."
                 << endl;
            break;
        }

        // Extract the inliers.
        extract.setInputCloud(filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        vector<int> filtered_idx;
        extract.filter(filtered_idx);
        int num_before = filtered->size();
        extract.filter(*filtered);
        int num_after = filtered->size();

        // Extraction creates unorganized point clouds, so track the index
        // mapping to preserve pixel coordinates.
        if (first_time) {
            first_time = false;
            keep_idx = filtered_idx;
        } else {
            vector<int> filt;
            for (int j : filtered_idx) {
                filt.push_back(keep_idx[j]);
            }
            swap(keep_idx, filt);
        }

        // Stop when the planes being removed become small.
        if (num_before - num_after < 20000) {
            break;
        }
    }

    if (indices_out != nullptr) {
        swap(*indices_out, keep_idx);
    }

    return filtered;
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
    auto new_end = remove_if(object_regions.begin(), object_regions.end(),
        [](const vector<Point2i>& region) {
            return region.size() < MIN_REGION_SIZE;
        }
    );
    object_regions.erase(new_end, object_regions.end());

    if (roi_out != nullptr)
        *roi_out = roi;

    return object_regions;
}
