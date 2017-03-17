#include "grasping_model.hpp"
#include <vector>
#include <queue>

using namespace cv;
using namespace std;

// TODO: convert from value in meters
static const uint16_t MIN_REX_DEPTH = 100;

struct GraspFeature {

};

static vector<Point2i> flood_select(const Mat& m, Point2i seed, Mat& visited) {
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

static vector<vector<Point2i>> find_nonzero_components(const Mat& m) {
    Mat visited(m.size(), CV_8UC1, 0);
    vector<vector<Point2i>> components;
    for (int y = 0; y < m.rows; ++y) {
        for (int x = 0; x < m.cols; ++x) {
            if (visited.at<uint8_t>(y,x) == 0 and m.at<uint16_t>(y,x) != 0) {
                components.push_back(flood_select(m, Point2i(x,y), visited));
            }
        }
    }
    return components;
}

GraspingPoints GraspingModel::search_grasping_points(
    const Mat& depth, const Mat& color)
{
    // Threshold and cull near-depth blobs. Blobs must be close enough to be
    // grabbed by the Rexarm (until MagicBot can drive itself).
    Mat near_depth;
    threshold(-depth, near_depth, -MIN_REX_DEPTH, 0, THRESH_TOZERO);
    vector<vector<Point2i>> object_regions =
        find_nonzero_components(near_depth);

    // Create feature vectors.
    vector<vector<GraspFeature>> features;

    // FEATURE 1: Multiscale texture energy vectors.
    Mat mean(color.size(), CV_8UC3);
    GaussianBlur(color, mean, Size(3,3), 0, 0);
    Mat mean_sub = color - mean;

    // FEATURE 2: Structure tensor.

    // FEATURE 3: Depth?

    // Do logistic regression.

    // Tranform into camera space.

    GraspingPoints points;
    return points;
}
