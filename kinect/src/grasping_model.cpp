#include "grasping_model.hpp"
#include <vector>
#include "img_proc.hpp"

using namespace cv;
using namespace std;

// TODO: get actual values for Rexarm
static const uint16_t MIN_REX_DEPTH = 800.0;
static const uint16_t MAX_REX_DEPTH = 1100.0;

struct GraspFeature {

};

GraspingPoints GraspingModel::search_grasping_points(
    const Mat& depth, const Mat& color)
{
    // Threshold and cull near-depth blobs. Blobs must be close enough to be
    // grabbed by the Rexarm (until MagicBot can drive itself).
    vector<vector<Point2i>> object_regions;

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
