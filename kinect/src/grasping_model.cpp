#include "grasping_model.hpp"
#include <vector>
#include "img_proc.hpp"

using namespace cv;
using namespace std;

// TODO: convert from value in meters
static const uint16_t MIN_REX_DEPTH = 100;

struct GraspFeature {

};

GraspingPoints GraspingModel::search_grasping_points(
    const Mat& depth, const Mat& color)
{
    // Threshold and cull near-depth blobs. Blobs must be close enough to be
    // grabbed by the Rexarm (until MagicBot can drive itself).
    vector<vector<Point2i>> object_regions =
        find_object_regions(depth, MIN_REX_DEPTH);

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
