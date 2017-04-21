#include "trash_search.hpp"
#include <Eigen/Core>

using namespace cv;
using namespace pcl;
using namespace std;
using namespace Eigen;

TrashSearch::TrashSearch(): state(RANDOM_WALK) {}

MCMotors feedback_control(
    const PointXYZ& medoid_pt, const vector<float>& ground_plane)
{
    return MCMotors();
}

TrashSearchState TrashSearch::get_state() const {
    return state;
}

bool TrashSearch::update(
    bool have_object,
    const Point3f& pickup_ftl,
    const Point3f& pickup_bbr,
    const PlaneInfo& plane_info,
    const PointXYZ& medoid_pt)
{
    if (plane_info.plane_eqs.size() > 1) {
        // Check if any of the non-ground planes are close to the robot.
        float min_plane_dist = 600.0;
        bool obstacle_near = false;
        for (size_t i = 1; i < plane_info.plane_eqs.size(); ++i) {
            if (abs(plane_info.plane_eqs[i][3]) < min_plane_dist) {
                obstacle_near = true;
                break;
            }
        }
        if (obstacle_near) {
            state = OBSTACLE_AVOIDANCE;
        } else if (state == OBSTACLE_AVOIDANCE) {
            state = RANDOM_WALK;
        }
    } else if (state == OBSTACLE_AVOIDANCE) {
        state = RANDOM_WALK;
    }

    if (state != OBSTACLE_AVOIDANCE) {
        if (state == FEEDBACK_CONTROL and have_object) {
            // Check if an object is in the workspace.
            // If so, go to PICKUP state.
            if (point_in_workspace(medoid_pt, pickup_ftl, pickup_bbr)) {
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
        motors = MCMotors(0.1, 0.1);
        break;
    case FEEDBACK_CONTROL:
        cout << "FEEDBACK CONTROL" << endl;
        motors = feedback_control(medoid_pt, plane_info.plane_eqs[0]);
        break;
    case OBSTACLE_AVOIDANCE:
        cout << "OBSTACLE AVOIDANCE" << endl;
        motors = MCMotors(-0.3, 0.3);
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
