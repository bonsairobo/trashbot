#ifndef OCCUPANCY_GRID_HPP
#define OCCUPANCY_GRID_HPP

#include <cstdint>
#include <opencv2/opencv.hpp>

class OccupancyGrid {
    uint8_t max_odds;
    uint8_t min_odds;
    uint8_t hit_odds;
    uint8_t miss_odds;
    cv::Mat odds;

    uint8_t clamp_odds(int odds);

public:
    OccupancyGrid(int width, int height);
    void update(
        const std::vector<std::vector<cv::Point2i>>& objects,
        const std::vector<cv::Point2i>& object_medoids,
        const std::vector<cv::Point2i>& edge_medoids);
    cv::Mat get_weights() const;
};

#endif // OCCUPANCY_GRID_HPP
