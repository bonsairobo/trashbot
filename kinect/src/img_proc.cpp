#include "img_proc.hpp"
#include <pcl/features/integral_image_normal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/pca.h>
#include <queue>
#include <cassert>
#include <Eigen/Core>

using namespace cv;
using namespace std;
using namespace openni;
using namespace pcl;
using namespace Eigen;
using namespace boost;

PlaneInfo merge_similar_planes(const PlaneInfo& plane_info) {
    // Cluster by normal dot product and distance.
    const float dist_thresh = 40.0;
    const float dot_diff_thresh = 0.05;
    vector<vector<float>> plane_eqs;
    vector<size_t> plane_sizes;

    // Always provide the ground plane. This will get merged in the case where
    // the ground plane is detected by segmentation. Otherwise, this estimate
    // will suffice.
    vector<float> ground_plane = { -0.02, 0.62, -0.78, 746.0 };
    plane_eqs.push_back(ground_plane);
    plane_sizes.push_back(1000); // dummy value

    int i = 0;
    for (const auto& plane_eq : plane_info.plane_eqs) {
        // Check for a similar plane.
        int close_cluster = -1;
        int j = 0;
        for (const auto& cluster_plane_eq : plane_eqs) {
            float dot = abs(plane_eq[0] * cluster_plane_eq[0]
                          + plane_eq[1] * cluster_plane_eq[1]
                          + plane_eq[2] * cluster_plane_eq[2]);
            float dist = abs(abs(cluster_plane_eq[3]) - abs(plane_eq[3]));
            if (abs(1.0 - dot) <= dot_diff_thresh and dist <= dist_thresh) {
                close_cluster = j;
                break;
            }
            ++j;
        }
        if (close_cluster == -1) {
            // Make a new cluster.
            plane_eqs.push_back(plane_eq);
            plane_sizes.push_back(plane_info.plane_sizes[i]);
        } else {
            // Take the bigger plane.
            if (plane_sizes[close_cluster] < plane_info.plane_sizes[i]) {
                plane_sizes[close_cluster] = plane_info.plane_sizes[i];
                plane_eqs[close_cluster] = plane_info.plane_eqs[i];
            }
        }
        ++i;
    }
    return { plane_eqs, plane_sizes };
}

Point2i region_centroid(const vector<Point2i>& region) {
    if (region.empty()) {
        return Point2i(0,0);
    }
    Point2i c;
    for (const auto& px : region) {
        c += px;
    }
    return c / float(region.size());
}

static Vector3f eigenvec3f_from_pointxyz(const PointXYZ& pt) {
    return Vector3f(pt.x, pt.y, pt.z);
}

static inline Vector3f cloud_centroid(
    vector<Point2i> object_px, PointCloud<PointXYZ>::Ptr cloud)
{
    if (cloud->empty()) {
        return Vector3f(0,0,0);
    }
    Vector3f c(0,0,0);
    for (auto& pt : *cloud) {
        c += eigenvec3f_from_pointxyz(pt);
    }
    return c / float(cloud->size());
}

static IndicesPtr indices_from_px(
    const vector<Point2i>& pixels, int width)
{
    IndicesPtr indices(new vector<int>);
    for (const auto& px : pixels) {
        indices->push_back(px.y * width + px.x);
    }
    return indices;
}

Vector3f object_principal_axis(
    vector<Point2i> object_px, PointCloud<PointXYZ>::Ptr cloud)
{
    pcl::PCA<PointXYZ> pca(true);
    pca.setInputCloud(cloud);
    IndicesPtr indices = indices_from_px(object_px, cloud->width);
    pca.setIndices(indices);
    return pca.getEigenVectors().row(0);
}

Point2i region_medoid(const vector<Point2i>& region) {
    Point2i c = region_centroid(region);
    float min_dist = numeric_limits<float>::max();
    Point2i medoid;
    for (const auto& px : region) {
        float dist = hypot(px.x - c.x, px.y - c.y);
        if (dist < min_dist) {
            min_dist = dist;
            medoid = px;
        }
    }
    return medoid;
}

vector<Point2i> translate_px_coords(
    const vector<Point2i>& coords, const Point2i& t)
{
    vector<Point2i> out;
    for (const auto& px : coords) {
        out.push_back(px + t);
    }
    return out;
}

Rect roi_from_workspace_corners(
    const Point3f& ftl, const Point3f& bbr, const VideoStream& depth_stream)
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
        uint16_t z; // unused
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
    VideoMode vm = depth_stream.getVideoMode();
    Rect roi(0, 0, vm.getResolutionX(), vm.getResolutionY());
    if (min_x > 0 and min_x < vm.getResolutionX())
        roi.x = min_x;
    if (min_y > 0 and min_y < vm.getResolutionY())
        roi.y = min_y;
    int width = max_x - min_x;
    int height = max_y - min_y;
    roi.width = min(vm.getResolutionX() - roi.x, width);
    roi.height = min(vm.getResolutionY() - roi.y, height);
    return roi;
}

void remove_small_regions(
    vector<vector<Point2i>> *object_regions, size_t min_region_size)
{
    auto new_end = remove_if(object_regions->begin(), object_regions->end(),
        [min_region_size](const vector<Point2i>& region) {
            return region.size() < min_region_size;
        }
    );
    object_regions->erase(new_end, object_regions->end());
}

// `idx_px_map` returns the original pixel coordinates as indices.
// I.e. for *returned* cloud point i,
//   px_coord[i] = (idx_px_map[i] % img_width, idx_px_map[i] / img_width)
static PlaneInfo remove_planes(
    PointCloud<PointXYZ>::Ptr pc,
    float dist_thresh,
    IndicesPtr idx_px_map)
{
    // Do plane segmentation.
    ModelCoefficients::Ptr coefficients(new ModelCoefficients);
    PointIndices::Ptr inliers(new PointIndices);
    SACSegmentation<PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setMaxIterations(500);
    seg.setDistanceThreshold(dist_thresh);
    ExtractIndices<PointXYZ> extract;
    vector<vector<float>> plane_eqs;
    vector<size_t> plane_sizes;

    while (true) {
        // Segment the largest planar component from the remaining cloud.
        seg.setInputCloud(pc);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            cerr << "Could not estimate a planar model for the given dataset."
                 << endl;
            break;
        }

        // Extract the inliers.
        extract.setInputCloud(pc);
        extract.setIndices(inliers);
        extract.setNegative(true);
        vector<int> kept_idx;
        extract.filter(kept_idx);
        int num_before = pc->size();
        extract.filter(*pc);
        int num_after = pc->size();

        // Extraction creates unorganized point clouds, so track the index
        // mapping to preserve pixel coordinates.
        vector<int> new_map;
        for (int j : kept_idx) {
            new_map.push_back((*idx_px_map)[j]);
        }
        swap(*idx_px_map, new_map);

        // Stop when the planes being removed become small.
        // TODO: Use inverse relationship between area and distance to improve
        // stopping accuracy (obstacle detection accuracy).
        if (num_before - num_after < 5000) {
            break;
        }

        plane_eqs.push_back(coefficients->values);
        plane_sizes.push_back(inliers->indices.size());
    }

    return { plane_eqs, plane_sizes };
}

static PointCloud<PointXYZ>::Ptr zero_cloud(int width, int height) {
    PointCloud<PointXYZ>::Ptr pc(new PointCloud<PointXYZ>);
    pc->width = width;
    pc->height = height;
    pc->resize(width * height);
    pc->is_dense = true;
    return pc;
}

bool point_in_workspace(
    const PointXYZ& pt, const Point3f& ftl, const Point3f& bbr)
{
    return pt.x > ftl.x and pt.x < bbr.x and
           pt.y < ftl.y and pt.y > bbr.y and
           pt.z > ftl.z and pt.z < bbr.z;
}

ObjectInfo get_workspace_objects(
    const VideoStream& depth_stream,
    const Mat& depth_f32_mat,
    const Point3f& ftl,
    const Point3f& bbr,
    const Rect& roi,
    size_t min_region_size,
    float plane_dist_thresh,
    float cluster_tolerance)
{
    // 2D workspace culling.
    Mat crop = depth_f32_mat(roi);
    threshold(crop, crop, ftl.z, 0, THRESH_TOZERO);
    threshold(crop, crop, bbr.z, 0, THRESH_TOZERO_INV);
    vector<vector<Point2i>> workspc_px = find_nonzero_components<float>(crop);
    remove_small_regions(&workspc_px, min_region_size);
    if (workspc_px.empty()) {
        return ObjectInfo();
    }

    // Create a point cloud of ROI regions.
    PointCloud<PointXYZ>::Ptr pc = zero_cloud(roi.width, roi.height);
    for (const auto& region : workspc_px) {
        for (const auto& px : region) {
            PointXYZ& pt = pc->at(px.x, px.y);
            CoordinateConverter::convertDepthToWorld(
                depth_stream,
                float(px.x+roi.x), float(px.y+roi.y),
                crop.at<float>(px),
                &pt.x, &pt.y, &pt.z);
        }
    }

    // 3D workspace culling. Organized cloud becomes unorganized, so keep track
    // of index -> pixel mapping.
    IndicesPtr idx_px_map(new vector<int>);
    for (size_t i = 0; i < pc->size(); ++i) {
        const auto& pt = pc->points[i];
        if (point_in_workspace(pt, ftl, bbr)) {
            idx_px_map->push_back(i);
        }
    }
    PointCloud<PointXYZ>::Ptr filt_pc = pc->makeShared();
    ExtractIndices<PointXYZ> filter;
    filter.setInputCloud(filt_pc);
    filter.setIndices(idx_px_map);
    filter.filter(*filt_pc);

    // Remove planes.
    PlaneInfo plane_info =
        remove_planes(filt_pc, plane_dist_thresh, idx_px_map);
    if (idx_px_map->size() < min_region_size) {
        return ObjectInfo();
    }

    // Cluster remaining points.
    vector<PointIndices> cluster_idx;
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
    tree->setInputCloud(filt_pc);
    EuclideanClusterExtraction<PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_region_size);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filt_pc);
    ec.extract(cluster_idx);

    // Convert clusters back into ROI-space pixel coordinates.
    vector<vector<Point2i>> object_px;
    for (const auto& cluster : cluster_idx) {
        vector<Point2i> px_coords;
        for (const auto& i : cluster.indices) {
            px_coords.push_back(Point2i(
                (*idx_px_map)[i] % roi.width, (*idx_px_map)[i] / roi.width));
        }
        object_px.push_back(px_coords);
    }

    return { plane_info, pc, object_px };
}

PointCloud<Normal>::Ptr estimate_normals(PointCloud<PointXYZ>::ConstPtr pc) {
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
    IntegralImageNormalEstimation<PointXYZ, Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(pc);
    ne.compute(*normals);
    return normals;
}
