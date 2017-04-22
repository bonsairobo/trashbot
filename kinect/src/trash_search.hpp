#ifndef TRASH_SEARCH_HPP
#define TRASH_SEARCH_HPP

#include "../../common/socket_types.hpp"
#include "img_proc.hpp"
#include <chrono>

enum TrashSearchState {
    NONE_STATE,
    RANDOM_WALK,
    FEEDBACK_WAIT,
    FEEDBACK_TURN,
    FEEDBACK_DRIVE,
    OBSTACLE_AVOIDANCE,
    PICKUP_WAIT,
    PICKUP
};

class TrashSearch {
    TrashSearchState state;

    // Time is kept in order to stay in certain states for a set amount of time.
    // Counting iterations would make the amount of drive time dependent on the
    // computational complexity of a given scene, which is not a great safety
    // characteristic.
    std::chrono::time_point<std::chrono::high_resolution_clock> prev_time;

public:
    TrashSearch(); // initialize state machine
    bool update(
        bool pickup_complete,
        bool have_object,
        const cv::Point3f& pickup_ftl,
        const cv::Point3f& pickup_bbr,
        const PlaneInfo&,
        const pcl::PointXYZ& medoid_pt);
    TrashSearchState get_state() const;

    MCMotors motors;
};

#endif // TRASH_SEARCH_HPP
