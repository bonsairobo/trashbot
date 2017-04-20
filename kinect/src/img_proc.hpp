#ifndef IMG_PROC_HPP
#define IMG_PROC_HPP

#include <opencv2/opencv.hpp>
#include <OpenNI.h>
#include <vector>
#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <common/socket_types.hpp>

struct PlaneInfo {
    std::vector<std::vector<float>> plane_eqs;
    std::vector<size_t> plane_sizes;
};

struct ObjectInfo {
    PlaneInfo plane_info;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::vector<std::vector<cv::Point2i>> object_pixels; // ROI coordinates
};

bool point_in_workspace(
    const pcl::PointXYZ&, const cv::Point3f& ftl, const cv::Point3f& bbr);

Eigen::Vector3f object_principal_axis(
    std::vector<cv::Point2i> object_px,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

void remove_small_regions(
    std::vector<std::vector<cv::Point2i>> *object_regions,
    size_t min_region_size);

cv::Rect roi_from_workspace_corners(
    const cv::Point3f& ftl,
    const cv::Point3f& bbr,
    const openni::VideoStream& depth_stream);

ObjectInfo get_workspace_objects(
    const openni::VideoStream& depth_stream,
    const cv::Mat& depth_f32_mat,
    const cv::Point3f& ftl,
    const cv::Point3f& bbr,
    const cv::Rect& roi,
    size_t min_region_size,
    float plane_dist_thresh,
    float cluster_tolerance);

cv::Point2i region_centroid(const std::vector<cv::Point2i>&);

cv::Point2i region_medoid(const std::vector<cv::Point2i>&);

std::vector<cv::Point2i> translate_px_coords(
    const std::vector<cv::Point2i>&,
    const cv::Point2i&);

pcl::PointCloud<pcl::Normal>::Ptr estimate_normals(
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr);

template<typename T, typename U>
cv::Mat mask_image(const cv::Mat& img, const cv::Mat& mask) {
    cv::Mat out = cv::Mat::zeros(img.size(), img.type());
    for (int y = 0; y < out.rows; ++y) {
        for (int x = 0; x < out.cols; ++x) {
            if (mask.at<T>(y,x) != 0) {
                out.at<U>(y,x) = img.at<U>(y,x);
            }
        }
    }
    return out;
}

template<typename T>
std::vector<cv::Point2i> flood_select(
    const cv::Mat& m,
    cv::Point2i seed,
    cv::Mat& visited)
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
    cv::Mat& img, const std::vector<cv::Point2i>& pixels, const T& color)
{
    for (auto px : pixels) {
        img.at<T>(px) = color;
    }
}

template<typename T>
void draw_points(
    cv::Mat& img, std::vector<cv::Point2i> pixels, const T& color)
{
    for (const auto& c : pixels) {
        std::vector<cv::Point2i> blob = {
            c,
            c + cv::Point2i(0, 1),
            c + cv::Point2i(1, 0),
            c + cv::Point2i(-1, 1),
            c + cv::Point2i(1, -1),
            c + cv::Point2i(0, -1),
            c + cv::Point2i(-1, 0),
            c + cv::Point2i(1, 1),
            c + cv::Point2i(-1, -1),
        };
        draw_pixels<T>(img, blob, color);
    }
}

inline Vec3f vec3f_from_eigen_vector3f(const Eigen::Vector3f v) {
    return { v(0), v(1), v(2) };
}

inline Vec3f vec3f_from_pointxyz(const pcl::PointXYZ& p) {
    return { p.x, p.y, p.z };
}

inline Vec3f vec3f_from_normal(const pcl::Normal& p) {
    return { p.normal_x, p.normal_y, p.normal_z };
}

#endif // IMG_PROC_HPP
