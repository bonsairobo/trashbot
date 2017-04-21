#include "occupancy_grid.hpp"
#include "img_proc.hpp"
#include <limits>

using namespace cv;
using namespace std;

OccupancyGrid::OccupancyGrid(int width, int height):
    height(height),
    width(width),
    max_odds(numeric_limits<uint8_t>::max()),
    min_odds(numeric_limits<uint8_t>::lowest()),
    hit_odds(20),
    miss_odds(5),
    odds(height, width, CV_8U, Scalar(max_odds / 2))
{}

void OccupancyGrid::reset() {
    odds = Scalar(0);
}

void OccupancyGrid::set_update_odds(uint8_t hit_odds, uint8_t miss_odds) {
    this->hit_odds = hit_odds;
    this->miss_odds = miss_odds;
}

uint8_t OccupancyGrid::clamp_odds(int odds) {
    if (odds < min_odds)
        return min_odds;
    if (odds > max_odds)
        return max_odds;
    return odds;
}

// Returns a score in [0,1] of how much the set of pixels resembles a blob
// rather than a noisy "ghost."
static float roi_object_score(const vector<Point2i>& region, const Mat& mask) {
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

void OccupancyGrid::update(
    const vector<vector<Point2i>>& objects,
    const vector<vector<Point2i>>& edges,
    const vector<Point2i>& object_medoids,
    const vector<Point2i>& edge_medoids)
{
    // Make object index image.
    Mat obj_idx_img(
        height, width, CV_16U, Scalar(numeric_limits<uint16_t>::max()));
    int i = 0;
    for (const auto& obj : objects) {
        draw_pixels<uint16_t>(obj_idx_img, obj, i);
        ++i;
    }

    // Rank objects by closest edge overlap percentage.
    const float min_pct = 0.15;
    vector<float> best_pct_edge(edges.size(), min_pct);
    vector<int> valid_obj_idx(edges.size(), -1);
    int obj_idx = 0;
    for (const auto& obj_med : object_medoids) {
        // Find closest edge.
        float min_dist = numeric_limits<float>::max();
        int j = 0;
        int edge_idx = -1;
        for (const auto& edge_med : edge_medoids) {
            float d = hypot(obj_med.x - edge_med.x, obj_med.y - edge_med.y);
            if (d < min_dist) {
                min_dist = d;
                edge_idx = j;
            }
            ++j;
        }

        // Calculate percentage of edge overlapped by object.
        int overlap = 0;
        for (const auto& px : edges[edge_idx]) {
            if (obj_idx_img.at<uint16_t>(px) == obj_idx) {
                ++overlap;
            }
        }
        float overlap_pct = float(overlap) / edges[edge_idx].size();

        // Take only the best percentage for each closest edge.
        if (overlap_pct > best_pct_edge[edge_idx]) {
            best_pct_edge[edge_idx] = overlap_pct;
            valid_obj_idx[edge_idx] = obj_idx;
        }

        ++obj_idx;
    }

    // Make padded object mask.
    Mat mask(odds.cols+2, odds.rows+2, CV_8U, Scalar(0));
    for (int i : valid_obj_idx) {
        if (i == -1) {
            continue;
        }
        const auto& obj = objects[i];
        for (const auto& px : obj) {
            mask.at<uint8_t>(px+Point(1,1)) = 1;
        }
    }
    // Hit object pixels.
    cv::Mat visited = cv::Mat::zeros(mask.size(), CV_8UC1);
    for (const auto& obj : objects) {
        float prob = roi_object_score(obj, mask);
        for (const auto& px : obj) {
            uint8_t& px_odds = odds.at<uint8_t>(px);
            px_odds = clamp_odds((int)px_odds + round(prob * hit_odds));
        }
        visited = Scalar(0);
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
