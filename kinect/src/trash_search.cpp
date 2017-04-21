#include "trash_search.hpp"
#include <Eigen/Core>

using namespace cv;
using namespace pcl;
using namespace std;
using namespace Eigen;

TrashSearch::TrashSearch(): state(RANDOM_WALK) {}

static MCMotors feedback_control(
    const PointXYZ& dst_pt, const vector<float>& ground_plane)
{
    // Calculate ground plane coordinates of destination point.
    Vector3f dst_pt_vec(dst_pt.x, dst_pt.y, dst_pt.z);
    Vector3f normal(ground_plane[0], ground_plane[1], ground_plane[2]);
    Vector3f x = Vector3f(1,0,0);
    Vector3f z = Vector3f(0,0,1);
    // (Vector in the plane is the rejection of the normal.)
    Vector3f ground_x = x - x.dot(normal) * normal;
    Vector3f ground_y = z - z.dot(normal) * normal;
    float ground_x_coord = ground_x.normalized().dot(dst_pt_vec);
    float ground_y_coord = ground_y.normalized().dot(dst_pt_vec);
    float angle = atan2(ground_y_coord, ground_x_coord) - 3.14159 / 2.0;

    // Drive toward the destination point.
    float turn_scalar = 0.6 * (2.0 / 3.14159);
    MCMotors motors(0.1, 0.1); // Base amplitudes.
    if (abs(angle > 0.35)) {
        motors.l_motor += -turn_scalar * angle;
        motors.r_motor += turn_scalar * angle;
    }
    return motors;
}

static MCMotors obstacle_avoidance(const vector<float>& obstacle_plane) {
    // Check the sign of the distance to get the normal orientation.
    // Use the X coordinate of the normal to decide which direction to turn.
    if (obstacle_plane[3] > 0) {
        if (obstacle_plane[0] > 0) {
            return MCMotors(0.3, -0.3);
        } else {
            return MCMotors(-0.3, 0.3);
        }
    } else {
        if (obstacle_plane[0] < 0) {
            return MCMotors(0.3, -0.3);
        } else {
            return MCMotors(-0.3, 0.3);
        }
    }
}

TrashSearchState TrashSearch::get_state() const {
    return state;
}

bool TrashSearch::update(
    bool pickup_complete,
    bool have_object,
    const Point3f& pickup_ftl,
    const Point3f& pickup_bbr,
    const PlaneInfo& plane_info,
    const PointXYZ& medoid_pt)
{
    vector<float> closest_plane;
    bool send_grasp = false;

    if (state != PICKUP or pickup_complete) {
        if (plane_info.plane_eqs.size() > 1) {
            // Check if any of the non-ground planes are close to the robot.
            float min_plane_dist = 700.0;
            for (size_t i = 1; i < plane_info.plane_eqs.size(); ++i) {
                if (abs(plane_info.plane_eqs[i][3]) < min_plane_dist) {
                    closest_plane = plane_info.plane_eqs[i];
                    break;
                }
            }
            if (!closest_plane.empty()) {
                state = OBSTACLE_AVOIDANCE;
            } else if (state == OBSTACLE_AVOIDANCE) {
                state = RANDOM_WALK;
            }
        } else if (state == OBSTACLE_AVOIDANCE) {
            state = RANDOM_WALK;
        }

        if (state != OBSTACLE_AVOIDANCE) {
            if (have_object) {
                if (state == FEEDBACK_CONTROL) {
                    // Check if an object is in the workspace.
                    // If so, go to PICKUP state.
                    if (point_in_workspace(medoid_pt, pickup_ftl, pickup_bbr)) {
                        send_grasp = true;
                        state = PICKUP;
                    }
                } else if (state == PICKUP) {
                    if (point_in_workspace(medoid_pt, pickup_ftl, pickup_bbr)) {
                        state = PICKUP;
                    }
                } else {
                    state = FEEDBACK_CONTROL;
                }
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
        motors = obstacle_avoidance(closest_plane);
        break;
    case PICKUP:
        cout << "PICKUP" << endl;
        motors = MCMotors();
        if (send_grasp) {
            return true;
        }
    case NONE_STATE:
        break;
    }

    return false;
}
