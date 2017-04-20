#include "trash_search.hpp"

using namespace cv;
using namespace pcl;
using namespace std;

TrashSearch::TrashSearch(): state(RANDOM_WALK) {}

void TrashSearch::update(
    const PlaneInfo& plane_info,
    const vector<vector<Point2i>>& objects,
    const vector<Point2i>& object_medoids,
    int best_obj_idx,
    PointCloud<PointXYZ>::ConstPtr cloud)
{
    if (plane_info.plane_eqs.size() > 1) {
        state = OBSTACLE_AVOIDANCE;
    }
}
