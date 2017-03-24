#include "img_proc.hpp"
#include "common.hpp"
#include <pcl/visualization/cloud_viewer.h>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <chrono>
#include <vector>

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

    namedWindow("world coordinates", 1);
    namedWindow("kinect_color", 1);
    namedWindow("webcam_color", 1);

    vector<fs::path> webcam_img_paths;
    fs::path p("images/" + string(argv[1]));
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

        // Do workspace pixel culling.
        Point3f ftl(-300.0, 300.0, 800.0);
        Point3f bbr(300.0, -300.0, 1200.0);
        Rect roi;
        vector<vector<Point2i>> workspc_px =
            get_workspace_pixels(depth_stream, depth_mat, ftl, bbr, &roi);
        Point2i tl_px = Point2i(roi.x, roi.y);

        // Create a point cloud of ROI regions.
        PointCloud<PointXYZ>::Ptr pc = zero_cloud(roi.width, roi.height);
        for (const auto& region : workspc_px) {
            for (const auto& px : region) {
                PointXYZ& pt = pc->at(px.x, px.y);
                CoordinateConverter::convertDepthToWorld(
                    depth_stream,
                    px.x+roi.x, px.y+roi.y,
                    depth_mat.at<uint16_t>(px.y+roi.y, px.x+roi.x),
                    &pt.x, &pt.y, &pt.z);
                pt.z *= -1.0;
            }
        }

        // Estimate normals.
        PointCloud<Normal>::Ptr normals = estimate_normals(pc);

        // Remove planes.
        vector<int> indices;
        PointCloud<PointXYZ>::Ptr object_pc = remove_planes(pc, &indices);
        vector<Point2i> object_px;
        for (int i : indices) {
            object_px.push_back(tl_px + Point2i(i % roi.width, i / roi.width));
        }

        // TODO: for plane outlier points, compute image features for logistic
        // regression grasping point classifier

        // Only draw the proposed object world coordinates.
        Mat coord_mat = Mat::zeros(depth_mat.size(), CV_32FC3);
        for (size_t i = 0; i < object_pc->size(); ++i) {
            PointXYZ pt = object_pc->at(i);
            Vec3f& coord = coord_mat.at<Vec3f>(object_px[i]);
            coord[0] = pt.x;
            coord[1] = pt.y;
            coord[2] = -pt.z;

            // TODO: remove, this is only for visualization
            coord *= 0.0005;
        }
        imshow("world coordinates", coord_mat);

        // Draw color and webcam.
        Mat masked = draw_color_on_depth(color_mat, depth_mat);
        draw_pixels(masked, object_px, Vec3b(200, 0, 0));
        imshow("kinect_color", masked);
        if (webcolor_mat.data) {
            imshow("webcam_color", webcolor_mat);
        }

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
