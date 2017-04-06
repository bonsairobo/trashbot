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
static float roi_object_score(
    const vector<Point2i>& region, Rect *roi_out)
{
    // TODO: refactor copy pasta from img_proc!
    int max_x = 0,
        max_y = 0,
        min_x = numeric_limits<int>::max(),
        min_y = numeric_limits<int>::max();
    for (const auto& px : region) {
        if (px.x < min_x) {
            min_x = px.x;
        }
        if (px.x > max_x) {
            max_x = px.x;
        }
        if (px.y < min_y) {
            min_y = px.y;
        }
        if (px.y > max_y) {
            max_y = px.y;
        }
    }
    int width = max_x - min_x;
    int height = max_y - min_y;
    if (roi_out != nullptr) {
        roi_out->x = min_x;
        roi_out->y = max_y;
        roi_out->width = width;
        roi_out->height = height;
    }

    // Draw padded region image.
    Mat img(height+2, width+2, CV_8U, Scalar(0));
    for (const auto& px : region) {
        img.at<uint8_t>(px - Point2i(min_x-1, min_y-1)) = 1;
    }

    // Calculate percentage of pixels touching an empty pixel.
    int num_adjacent_empty = 0;
    for (const auto& px : region) {
        bool stop = false;
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                if (i == 0 and j == 0)
                    continue;

                if (img.at<uint8_t>(
                    px - Point2i(min_x-1, min_y-1) + Point2i(i,j)) == 0)
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
    // Hit object pixels.
    Mat touched(odds.cols, odds.rows, CV_8U, Scalar(0));
    for (const auto& obj : objects) {
        float prob = roi_object_score(obj, nullptr);
        for (const auto& px : obj) {
            uint8_t& px_odds = odds.at<uint8_t>(px);
            px_odds = clamp_odds((int)px_odds + round(prob * hit_odds));
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
