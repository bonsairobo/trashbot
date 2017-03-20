#include "img_proc.hpp"
#include "common.hpp"
#include <boost/filesystem.hpp>
#include <algorithm>
#include <chrono>

namespace fs = boost::filesystem;

using namespace std;
using namespace cv;
using namespace openni;

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

    namedWindow("world coordinates", 1);
    namedWindow("kinect_color", 1);
    namedWindow("webcam_color", 1);

    vector<fs::path> webcam_img_paths;
    fs::path p("images");
    if (fs::is_directory(p)) {
        copy(fs::directory_iterator(p),
            fs::directory_iterator(),
            back_inserter(webcam_img_paths));
    }
    sort(webcam_img_paths.begin(), webcam_img_paths.end());

    // Seek through recording by depth frame index.
    PlaybackControl *pbc = device.getPlaybackControl();
    pbc->setSpeed(-1); // Make sure reading never blocks.

    // Interpolation between Kinect and webcam streams.
    int num_depth_frames = pbc->getNumberOfFrames(depth_stream);
    int num_color_frames = pbc->getNumberOfFrames(color_stream);
    int num_webcam_frames = webcam_img_paths.size();
    cout << "# DEPTH FRAMES = " << num_depth_frames << endl;
    cout << "# COLOR FRAMES = " << num_color_frames << endl;
    cout << "# WEBCAM FRAMES = " << num_webcam_frames << endl;
    float web_per_depth = float(num_webcam_frames) / float(num_depth_frames);

    for (int i = 0; i < num_depth_frames; ++i) {
        // This should set also set color stream to the same point in time.
        pbc->seek(depth_stream, i);

        Mat depth_mat, color_mat;
        VideoFrameRef depth_frame;
        if (get_mat_from_stream(
            depth_stream, depth_mat, cout, 2, &depth_frame) != 0)
        {
            return 1;
        }
        if (get_mat_from_stream(color_stream, color_mat, cout, 3, nullptr) != 0)
        {
            return 1;
        }

        // Load the corresponding webcam image.
        Mat webcolor_mat = imread(
            webcam_img_paths[round(i * web_per_depth)].string());

        // Do image analysis.
        Mat masked = draw_color_on_depth(color_mat, depth_mat);
        vector<vector<Point2i>> regions =
            find_object_regions(depth_mat, 800.0, 1100.0);
        cout << "Found " << regions.size() << " regions" << endl;
        for (auto& region : regions) {
            draw_pixels(masked, region, Vec3b(200, 0, 0));
        }
        imshow("kinect_color", masked);
        if (webcolor_mat.data) {
            imshow("webcam_color", webcolor_mat);
        }

        // Time how long it takes to create a world coordinate image.
        auto start = chrono::system_clock::now();
        Mat coord_mat = Mat::zeros(depth_mat.size(), CV_32FC3);
        for (int u = 0; u < depth_mat.rows; ++u) {
            for (int v = 0; v < depth_mat.cols; ++v) {
                Vec3f& coord = coord_mat.at<Vec3f>(u,v);
                CoordinateConverter::convertDepthToWorld(
                    depth_stream, v, u, depth_mat.at<uint16_t>(u, v),
                    &coord[0], &coord[1], &coord[2]);
                coord *= 0.0005;
            }
        }
        auto end = chrono::system_clock::now();
        chrono::duration<double> diff = end - start;
        cout << "Time to convert pixels: " << diff.count() << endl;
        imshow("world coordinates", coord_mat);

        char key = waitKey(0);
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
