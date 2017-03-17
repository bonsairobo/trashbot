#include "img_proc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    if (argc < 2) {
        cerr << "Need the number of images to read!" << endl;
        return 1;
    }
    int num_images = atoi(argv[1]);
    for (int i = 0; i < num_images; ++i) {
        Mat depth = imread("images/depth" + to_string(i) + ".jpg");
        depth.convertTo(depth, CV_16UC1);
        Mat color = imread("images/color" + to_string(i) + ".jpg");
        //Mat rgbd = draw_color_on_depth(color, depth);
        //imwrite("images/masked" + to_string(i) + ".jpg", rgbd);

        Mat depth_f32;
        depth.convertTo(depth_f32, CV_32FC1);
        vector<vector<Point2i>> object_regions =
            find_object_regions(depth_f32, 1000);
        Mat region_img = color.clone();
        for (const auto& region : object_regions) {
            draw_pixels(region_img, region, Scalar(200, 0, 0));
        }
        imwrite("images/region" + to_string(i) + ".jpg", region_img);
    }
    return 0;
}
