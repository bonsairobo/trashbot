#include "img_proc.hpp"
#include "common.hpp"
#include <pcl/visualization/cloud_viewer.h>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <chrono>
#include <vector>
#include <random>

namespace fs = boost::filesystem;

using namespace std;
using namespace cv;
using namespace openni;
using namespace pcl;

int main(int argc, char **argv) {
    if (argc < 2) {
        cerr << "Need the ONI file location!" << endl;
        return 1;
    }

    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK) {
        cerr << "Initialize failed" << endl
             << OpenNI::getExtendedError() << endl;
        return 1;
    }

    Device device;
    rc = device.open(argv[1]);
    if (rc != STATUS_OK) {
        cerr << "Couldn't open device" << endl
             << OpenNI::getExtendedError() << endl;
        return 1;
    }

    VideoStream depth_stream, color_stream;
    if (try_start_rgbd_streams(
        device, depth_stream, color_stream, cout, false) != 0)
    {
        return 1;
    }

    namedWindow("kinect", 1);
    //namedWindow("webcam", 1);

    /*vector<fs::path> webcam_img_paths;
    fs::path path("images/" + string(argv[1]));
    if (fs::is_directory(path)) {
        copy(fs::directory_iterator(path),
            fs::directory_iterator(),
            back_inserter(webcam_img_paths));
    }
    sort(webcam_img_paths.begin(), webcam_img_paths.end());*/

    // Seek through recording by depth frame index.
    PlaybackControl *pbc = device.getPlaybackControl();
    pbc->setSpeed(-1); // Make sure reading never blocks.

    // Interpolation between Kinect and webcam streams.
    int num_depth_frames = pbc->getNumberOfFrames(depth_stream);
    int num_color_frames = pbc->getNumberOfFrames(color_stream);
    //int num_webcam_frames = webcam_img_paths.size();
    cout << "# DEPTH FRAMES = " << num_depth_frames << endl;
    cout << "# COLOR FRAMES = " << num_color_frames << endl;
    //cout << "# WEBCAM FRAMES = " << num_webcam_frames << endl;
    //float web_per_depth = float(num_webcam_frames) / float(num_depth_frames);

    // Create RNG for RGB values.
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint8_t> dis(100, 200);

    for (int i = 0; i < num_depth_frames; ++i) {
        // This should set also set color stream to the same point in time.
        pbc->seek(depth_stream, i);

        Mat depth_u16_mat, color_mat;
        VideoFrameRef depth_frame;
        if (get_mat_from_stream(
            depth_stream, depth_u16_mat, cout, 2, &depth_frame) != 0)
        {
            return 1;
        }
        if (get_mat_from_stream(color_stream, color_mat, cout, 3, nullptr) != 0)
        {
            return 1;
        }

        Mat depth_f32_mat;
        depth_u16_mat.convertTo(depth_f32_mat, CV_32F);

        // Load the corresponding webcam image.
        /*Mat webcolor_mat = imread(
            webcam_img_paths[round(i * web_per_depth)].string());*/

        // Get pixels, points, normals, and ROI for workspace objects.
        ObjectInfo obj_info =
            get_workspace_objects(depth_stream, depth_f32_mat);
        Point2i tl_px(obj_info.roi.x, obj_info.roi.y);

        // TODO: see if normal estimation would benefit from an optimization
        // that cuts the cloud image into sub-images, one for each object.
        auto normal_cloud = estimate_normals(obj_info.cloud);

        // Choose the "best" object.
        // TODO: make this smarter.
        float min_depth = numeric_limits<float>::max();
        int best_obj_idx = -1;
        int j = 0;
        for (const auto& object : obj_info.object_pixels) {
            auto px = object[0];
            float z = -obj_info.cloud->at(px.x, px.y).z;
            if (z < min_depth) {
                min_depth = z;
                best_obj_idx = j;
            }
            ++j;
        }

        // TODO: for object pixels, compute image features for logistic
        // regression grasping point classifier

        // Draw color and webcam.
        Mat masked = draw_color_on_depth(color_mat, depth_u16_mat);
        j = 0;
        for (const auto& object : obj_info.object_pixels) {
            Vec3b color = j == best_obj_idx ?
                Vec3b(0, 0, 255) :
                Vec3b(dis(gen), dis(gen), dis(gen));
            draw_pixels(masked, translate_px_coords(object, tl_px), color);
            ++j;
        }
        imshow("kinect", masked);
        /*if (webcolor_mat.data) {
            imshow("webcam", webcolor_mat);
        }*/

        char key = waitKey(1);
        if (key == 27) {
            break;
        }
    }

    color_stream.stop();
    color_stream.destroy();
    depth_stream.stop();
    depth_stream.destroy();
    device.close();
    OpenNI::shutdown();
    return 0;
}
