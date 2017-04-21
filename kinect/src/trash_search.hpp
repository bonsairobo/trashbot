#ifndef TRASH_SEARCH_HPP
#define TRASH_SEARCH_HPP

#include "../../common/socket_types.hpp"
#include "img_proc.hpp"

enum TrashSearchState {
    NONE_STATE,
    RANDOM_WALK,
    FEEDBACK_CONTROL,
    OBSTACLE_AVOIDANCE,
    PICKUP
};

class TrashSearch {
    TrashSearchState state;

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
