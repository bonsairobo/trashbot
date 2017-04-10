#include "img_proc.hpp"
#include "common.hpp"
#include "occupancy_grid.hpp"
#include <vector>
#include <random>

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
    namedWindow("object_grid", 1);
    namedWindow("edges", 1);

    // Seek through recording by depth frame index.
    PlaybackControl *pbc = device.getPlaybackControl();
    pbc->setSpeed(-1); // Make sure reading never blocks.

    int num_depth_frames = pbc->getNumberOfFrames(depth_stream);
    int num_color_frames = pbc->getNumberOfFrames(color_stream);
    cout << "# DEPTH FRAMES = " << num_depth_frames << endl;
    cout << "# COLOR FRAMES = " << num_color_frames << endl;

    // Create RNG for RGB values.
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint8_t> dis(100, 200);

    VideoMode vm = depth_stream.getVideoMode();
    OccupancyGrid object_grid(vm.getResolutionX(), vm.getResolutionY());

    Point3f ftl(-200.0, 0.0, 650.0);
    Point3f bbr(200.0, -280.0, 850.0);

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

        // Get pixels, points, normals, and ROI for workspace objects.
        ObjectInfo obj_info = get_workspace_objects(
            depth_stream, depth_f32_mat, ftl, bbr, 100, 2.5, 4.3);
        Point2i tl_px(obj_info.roi.x, obj_info.roi.y);

        // Choose the closest object to the Rexarm.
        float min_dist = numeric_limits<float>::max();
        int best_obj_idx = -1;
        int j = 0;
        for (const auto& object : obj_info.object_pixels) {
            auto px = object[0];
            PointXYZ pt = obj_info.cloud->at(px.x, px.y);
            float d = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            if (d < min_dist) {
                min_dist = d;
                best_obj_idx = j;
            }
            ++j;
        }

        // Translate pixel coordinates back to original image from ROI.
        vector<vector<Point2i>> trans_object_px;
        for (const auto& object : obj_info.object_pixels) {
            trans_object_px.push_back(translate_px_coords(object, tl_px));
        }

        // Draw object clusters on color image.
        Mat masked = draw_color_on_depth(color_mat, depth_u16_mat);
        j = 0;
        for (const auto& object : trans_object_px) {
            Vec3b color = j == best_obj_idx ?
                Vec3b(0, 0, 255) :
                Vec3b(dis(gen), dis(gen), dis(gen));
            draw_pixels(masked, object, color);
            ++j;
        }
        imshow("kinect", masked);

        Mat gray_mat, edges;
        cvtColor(color_mat, gray_mat, CV_BGR2GRAY);
        blur(gray_mat, edges, Size(3,3));
        Canny(edges, edges, 50, 150, 3);
        int dilation_size = 1;
        Mat element = getStructuringElement(
            MORPH_RECT,
            Size(2 * dilation_size + 1, 2 * dilation_size + 1),
            Point(dilation_size, dilation_size));
        dilate(edges, edges, element);

        // Extract objects from edges in point cloud ROI.
        vector<vector<Point2i>> edge_objects =
            find_nonzero_components<uint8_t>(edges);

        vector<Point2i> object_medoids;
        for (const auto& obj : trans_object_px) {
            object_medoids.push_back(region_medoid(obj));
        }
        vector<Point2i> edge_medoids;
        for (const auto& obj : edge_objects) {
            edge_medoids.push_back(region_medoid(obj));
        }

        Mat color_edges;
        cvtColor(edges, color_edges, CV_GRAY2BGR);
        draw_points(color_edges, edge_medoids, Vec3b(0,0,255));
        draw_points(color_edges, object_medoids, Vec3b(255,0,0));
        imshow("edges", color_edges);

        object_grid.update(trans_object_px, object_medoids, edge_medoids);
        Mat weights = object_grid.get_weights();
        threshold(weights, weights, 0.8, 0, THRESH_TOZERO);
        imshow("object_grid", weights);

        // Finally, now that transients are removed, do one final clustering
        // of object points.

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
