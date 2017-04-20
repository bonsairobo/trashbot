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
        bool have_object,
        const cv::Point3f& pickup_ftl,
        const cv::Point3f& pickup_bbr,
        const PlaneInfo&,
        const cv::Point2i& object_medoid,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

    MCMotors motors;
};

#endif // TRASH_SEARCH_HPP
