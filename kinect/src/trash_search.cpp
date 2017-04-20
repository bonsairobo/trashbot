#include "trash_search.hpp"

using namespace cv;
using namespace pcl;
using namespace std;

TrashSearch::TrashSearch(): state(RANDOM_WALK) {}

MCMotors random_walk() {
    return MCMotors();
}

MCMotors obstacle_avoidance() {
    return MCMotors();
}

MCMotors feedback_control() {
    return MCMotors();
}

bool TrashSearch::update(
    bool have_object,
    const Point3f& pickup_ftl,
    const Point3f& pickup_bbr,
    const PlaneInfo& plane_info,
    const Point2i& object_medoid,
    PointCloud<PointXYZ>::ConstPtr cloud)
{
    if (plane_info.plane_eqs.size() > 1) {
        // Check if any of the non-ground planes are close to the robot.
        float min_plane_dist = 700.0;
        bool obstacle_near = false;
        for (size_t i = 1; i < plane_info.plane_eqs.size(); ++i) {
            if (plane_info.plane_eqs[i][3] < min_plane_dist) {
                obstacle_near = true;
                break;
            }
        }
        if (obstacle_near) {
            state = OBSTACLE_AVOIDANCE;
        }
    }

    if (state != OBSTACLE_AVOIDANCE) {
        if (state == FEEDBACK_CONTROL and have_object) {
            // Check if an object is in the workspace.
            // If so, go to PICKUP state.
            PointXYZ pt = cloud->at(object_medoid.x, object_medoid.y);
            if (point_in_workspace(pt, pickup_ftl, pickup_bbr)) {
                state = PICKUP;
            }
        } else {
            if (have_object) {
                state = FEEDBACK_CONTROL;
            } else {
                state = RANDOM_WALK;
            }
        }
    }

    switch (state) {
    case RANDOM_WALK:
        cout << "RANDOM WALK" << endl;
        motors = random_walk();
        break;
    case FEEDBACK_CONTROL:
        cout << "FEEDBACK CONTROL" << endl;
        motors = feedback_control();
        break;
    case OBSTACLE_AVOIDANCE:
        cout << "OBSTACLE AVOIDANCE" << endl;
        motors = obstacle_avoidance();
        break;
    case PICKUP:
        cout << "PICKUP" << endl;
        motors = MCMotors();
        return true;
    case NONE_STATE:
        break;
    }

    return false;
}
