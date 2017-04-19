#include "trash_search.hpp"

TrashSearch::TrashSearch(): state(RANDOM_WALK) {}

void TrashSearch::update(const PlaneInfo& plane_info) {
    if (plane_info.plane_eqs.size() > 1) {
        state = OBSTACLE_AVOIDANCE;
    }
}
