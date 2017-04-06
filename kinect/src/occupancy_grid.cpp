#include "occupancy_grid.hpp"
#include <limits>

using namespace cv;
using namespace std;

OccupancyGrid::OccupancyGrid(int width, int height):
    max_odds(numeric_limits<uint8_t>::max()),
    min_odds(numeric_limits<uint8_t>::lowest()),
    hit_odds(10),
    miss_odds(3),
    odds(height, width, CV_8U, Scalar(max_odds / 2))
{}

uint8_t OccupancyGrid::clamp_odds(int odds) {
    if (odds < min_odds)
        return min_odds;
    if (odds > max_odds)
        return max_odds;
    return odds;
}

// Returns a score in [0,1] of how much the set of pixels resembles a blob
// rather than a noisy "ghost."
static float roi_object_score(const vector<Point2i>& region, const Mat& mask)
{
    // Calculate percentage of pixels touching an empty pixel.
    int num_adjacent_empty = 0;
    bool stop = false;
    for (const auto& px : region) {
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                if (i == 0 and j == 0)
                    continue;

                if (mask.at<uint8_t>(
                    px + Point2i(i+1,j+1)) == 0)
                {
                    ++num_adjacent_empty;
                    stop = true;
                    break;
                }
            }
            if (stop)
                break;
        }
    }
    return 1.0 - float(num_adjacent_empty) / region.size();
}

void OccupancyGrid::update(const vector<vector<Point2i>>& objects) {
    // Make padded object mask.
    Mat mask(odds.cols+2, odds.rows+2, CV_8U, Scalar(0));
    for (const auto& obj : objects) {
        for (const auto& px : obj) {
            mask.at<uint8_t>(px+Point(1,1)) = 1;
        }
    }
    // Hit object pixels.
    for (const auto& obj : objects) {
        float prob = roi_object_score(obj, mask);
        for (const auto& px : obj) {
            uint8_t& px_odds = odds.at<uint8_t>(px);
            px_odds = clamp_odds((int)px_odds + round(prob * hit_odds));
        }
    }
    // Miss non-object pixels.
    for (int i = 0; i < odds.rows; ++i) {
        for (int j = 0; j < odds.cols; ++j) {
            if (mask.at<uint8_t>(i+1,j+1) == 0) {
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
