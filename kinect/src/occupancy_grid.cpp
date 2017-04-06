#include "occupancy_grid.hpp"
#include <limits>

using namespace cv;
using namespace std;

OccupancyGrid::OccupancyGrid(int width, int height):
    max_odds(numeric_limits<uint8_t>::max()),
    min_odds(numeric_limits<uint8_t>::lowest()),
    hit_odds(15),
    miss_odds(10),
    odds(height, width, CV_8U, Scalar(max_odds / 2))
{}

uint8_t OccupancyGrid::clamp_odds(int odds) {
    if (odds < min_odds)
        return min_odds;
    if (odds > max_odds)
        return max_odds;
    return odds;
}

void OccupancyGrid::update(const vector<vector<Point2i>>& objects) {
    // Hit object pixels.
    Mat touched(odds.cols, odds.rows, CV_8U, Scalar(0));
    for (const auto& obj : objects) {
        for (const auto& px : obj) {
            uint8_t& px_odds = odds.at<uint8_t>(px);
            px_odds = clamp_odds((int)px_odds + hit_odds);
            touched.at<uint8_t>(px) = 1;
        }
    }
    // Miss non-object pixels.
    for (int i = 0; i < odds.rows; ++i) {
        for (int j = 0; j < odds.cols; ++j) {
            if (touched.at<uint8_t>(i,j) == 0) {
                uint8_t& px_odds = odds.at<uint8_t>(i,j);
                px_odds = clamp_odds((int)px_odds - miss_odds);
            }
        }
    }
}

Mat OccupancyGrid::get_weights() const {
    Mat odds_f32;
    odds.convertTo(odds_f32, CV_32F);
    return (odds_f32 - min_odds) / (max_odds - min_odds);
}
