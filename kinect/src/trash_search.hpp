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
    void update(
        const PlaneInfo&,
        const std::vector<std::vector<cv::Point2i>>& objects,
        const std::vector<cv::Point2i>& object_medoids,
        int best_object_idx,
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

    MCMotors motors;
};

#endif // TRASH_SEARCH_HPP
