#include "trash_search.hpp"
#include <Eigen/Core>

using namespace cv;
using namespace pcl;
using namespace std;
using namespace chrono;
using namespace Eigen;

TrashSearch::TrashSearch(): state(RANDOM_WALK) {}

static float angle_to_point(
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
    return atan2(ground_y_coord, ground_x_coord) - 3.14159 / 2.0;
}

static MCMotors feedback_turn(
    const PointXYZ& dst_pt, const vector<float>& ground_plane)
{
    float angle = angle_to_point(dst_pt, ground_plane);
    cout << "ANGLE TO OBJECT = " << angle << endl;
    float turn_scalar = 2.0 * (2.0 / 3.14159);
    return MCMotors(-turn_scalar * angle, turn_scalar * angle);
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
    const PointXYZ& dst_pt)
{
    const float pickup_wait_time = 1.0;
    const float feedback_wait_time = 0.8;
    const float feedback_drive_time = 0.8;
    const float feedback_turn_time = 0.1;
    float min_plane_dist = state == RANDOM_WALK ? 650.0 : 550.0;

    vector<float> closest_plane;
    if (plane_info.plane_eqs.size() > 1) {
        // Check if any of the non-ground planes are close to the robot.
        for (size_t i = 1; i < plane_info.plane_eqs.size(); ++i) {
            if (abs(plane_info.plane_eqs[i][3]) < min_plane_dist) {
                closest_plane = plane_info.plane_eqs[i];
                break;
            }
        }
    }

    auto time_now = high_resolution_clock::now();
    duration<float> time_span =
        duration_cast<duration<float>>(time_now - prev_time);

    bool send_grasp = false;
    if (!closest_plane.empty()) {
        // Obstacle avoidance is the first priority.
        state = OBSTACLE_AVOIDANCE;
    } else if (state == OBSTACLE_AVOIDANCE) {
        state = RANDOM_WALK;
    } else if (state == RANDOM_WALK) {
        if (have_object) {
            state = FEEDBACK_WAIT;
            prev_time = time_now;
        }
    } else if (state == FEEDBACK_WAIT) {
        if (time_span.count() >= feedback_wait_time) {
            if (have_object) {
                prev_time = time_now;
                if (point_in_workspace(dst_pt, pickup_ftl, pickup_bbr)) {
                    state = PICKUP_WAIT;
                } else {
                    float angle =
                        angle_to_point(dst_pt, plane_info.plane_eqs[0]);
                    if (abs(angle) > 0.2) {
                        state = FEEDBACK_TURN;
                    } else {
                        state = FEEDBACK_DRIVE;
                    }
                }
            } else {
                state = RANDOM_WALK;
            }
        }
    } else if (state == FEEDBACK_DRIVE) {
        if (time_span.count() >= feedback_drive_time) {
            prev_time = time_now;
            state = FEEDBACK_WAIT;
        }
    } else if (state == FEEDBACK_TURN) {
        if (time_span.count() >= feedback_turn_time) {
            prev_time = time_now;
            state = FEEDBACK_WAIT;
        }
    } else if (state == PICKUP_WAIT) {
        if (time_span.count() >= pickup_wait_time) {
            prev_time = time_now;
            if (have_object) {
                if (point_in_workspace(dst_pt, pickup_ftl, pickup_bbr)) {
                    send_grasp = true;
                    state = PICKUP;
                } else {
                    state = FEEDBACK_WAIT;
                }
            } else {
                state = RANDOM_WALK;
            }
        }
    } else if (state == PICKUP) {
        if (pickup_complete) {
            state = RANDOM_WALK;
        }
    }

    switch (state) {
    case OBSTACLE_AVOIDANCE:
        cout << "OBSTACLE AVOIDANCE" << endl;
        motors = obstacle_avoidance(closest_plane);
        break;
    case RANDOM_WALK:
        cout << "RANDOM WALK" << endl;
        motors = MCMotors(0.1, 0.1);
        break;
    case FEEDBACK_WAIT:
        cout << "FEEDBACK WAIT" << endl;
        motors = MCMotors();
        break;
    case FEEDBACK_TURN:
        cout << "FEEDBACK TURN" << endl;
        motors = feedback_turn(dst_pt, plane_info.plane_eqs[0]);
        break;
    case FEEDBACK_DRIVE:
        cout << "FEEDBACK DRIVE" << endl;
        motors = MCMotors(0.1, 0.1);
        break;
    case PICKUP_WAIT:
        cout << "PICKUP WAIT" << endl;
        motors = MCMotors();
        break;
    case PICKUP:
        cout << "PICKUP" << endl;
        motors = MCMotors();
        if (send_grasp) {
            return true;
        }
        break;
    case NONE_STATE:
        break;
    }

    return false;
}
