#include "occupancy_grid.hpp"
#include "img_proc.hpp"
#include <limits>

using namespace cv;
using namespace std;

OccupancyGrid::OccupancyGrid(int width, int height):
    max_odds(numeric_limits<uint8_t>::max()),
    min_odds(numeric_limits<uint8_t>::lowest()),
    hit_odds(25),
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
    const vector<vector<Point2i>>& objects, const Mat& edges)
{
    // Extract objects from edges in point cloud ROI.
    vector<vector<Point2i>> edge_objects =
        find_nonzero_components<uint8_t>(edges);

    // Compare centroids of edge objects and point cloud objects.
    vector<Point2i> edge_centroids;
    for (const auto& obj : edge_objects) {
        edge_centroids.push_back(region_centroid(obj));
    }
    vector<Point2i> object_centroids;
    for (const auto& obj : objects) {
        object_centroids.push_back(region_centroid(obj));
    }
    const float min_dist = 25.0;
    int obj_idx = 0;
    vector<int> valid_obj_idx;
    for (const auto& obj_cent : object_centroids) {
        for (const auto& edge_cent : edge_centroids) {
            if (hypot(obj_cent.x - edge_cent.x, obj_cent.y - edge_cent.y)
                <= min_dist)
            {
                valid_obj_idx.push_back(obj_idx);
                break;
            }
        }
        ++obj_idx;
    }

    // Make padded object mask.
    Mat mask(odds.cols+2, odds.rows+2, CV_8U, Scalar(0));
    for (int i : valid_obj_idx) {
        const auto& obj = objects[i];
        for (const auto& px : obj) {
            mask.at<uint8_t>(px+Point(1,1)) = 1;
        }
    }
    // Hit object pixels.
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, 100000);
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
