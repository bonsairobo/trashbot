#include "../../common/socket_types.hpp"
#include "img_proc.hpp"
#include "common.hpp"
#include "occupancy_grid.hpp"
#include <fstream>
#include <random>

using namespace std;
using namespace openni;
using namespace cv;
using namespace pcl;

static const uint8_t ESC_KEYCODE = 27;

int main(int argc, char **argv) {
    // Open windows for drawing streams.
    bool show_feeds = !(argc > 1 and *(argv[1]) == '0');

    // Saves all frames from depth and color streams in ONI files.
    bool record_streams = argc > 2 and *(argv[2]) == '1';

    // All debug and error messages get saved. `watch` the file for
    // realtime feedback.
    ofstream log_stream("kinect_log.txt");

    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK) {
        log_stream << "Initialize failed" << endl
                   << OpenNI::getExtendedError() << endl;
        return 1;
    }

    Device device;
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK) {
        log_stream << "Couldn't open device" << endl
                   << OpenNI::getExtendedError() << endl;
        return 1;
    }

    VideoStream depth_stream, color_stream;
    if (try_start_rgbd_streams(
        device, depth_stream, color_stream, log_stream, true) != 0)
    {
        return 1;
    }

    Recorder recorder;
    if (record_streams) {
        recorder.create("./rgbd_stream.ONI");
        recorder.attach(color_stream);
        recorder.attach(depth_stream);
        recorder.start();
    }

    if (show_feeds) {
        namedWindow("kinect_feed", 1);
        namedWindow("objects", 1);
    }

    // Create RNG for RGB values.
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint8_t> dis(100, 200);

    // Set up UDP socket for receiving command from joystick and sending data
    // to Rexarm.
    //sockaddr_un js_addr = create_udp_addr("/tmp/joystick_endpoint");
    sockaddr_un kin_addr = create_udp_addr("/tmp/kinect_endpoint");
    sockaddr_un rex_addr = create_udp_addr("/tmp/rexarm_endpoint");
    int sock = try_create_udp_socket();
    try_bind_path(sock, kin_addr);

    VideoMode vm = depth_stream.getVideoMode();
    OccupancyGrid object_grid(vm.getResolutionX(), vm.getResolutionY());

    Point3f ftl(-300.0, 300.0, 650.0);
    Point3f bbr(300.0, -280.0, 850.0);

    uint8_t key = 0;
    while (key != ESC_KEYCODE) {
        Mat depth_u16_mat, color_mat;
        VideoFrameRef depth_frame;
        if (get_mat_from_stream(
            depth_stream, depth_u16_mat, log_stream, 2, &depth_frame) != 0)
        {
            return 1;
        }
        if (get_mat_from_stream(
            color_stream, color_mat, log_stream, 3, nullptr) != 0)
        {
            return 1;
        }

        Mat depth_f32_mat;
        depth_u16_mat.convertTo(depth_f32_mat, CV_32F);

        // Check for command to search for grasping points.
        /*PickupCommand cmd;
        socklen_t len = sizeof(js_addr);
        ssize_t bytes_read = 1;*/
        bool do_search = true;
        /*while (bytes_read > 0) {
            bytes_read = recvfrom(
                sock,
                &cmd,
                sizeof(cmd),
                0,
                (sockaddr*)&js_addr,
                &len);
            if (bytes_read < 0) {
                // TODO: remove this hack
                //perror("recvfrom");
                continue;
                //exit(1);
            } else if (bytes_read != sizeof(cmd)) {
                continue;
            }

            // Consume all commands and only do a single search.
            do_search = true;
        }*/

        if (do_search) {
            // Get pixels, cloud, and ROI for workspace objects.
            ObjectInfo obj_info = get_workspace_objects(
                depth_stream, depth_f32_mat, ftl, bbr, 100, 3.0, 4.3);
            if (!obj_info.object_pixels.empty()) {
                // Choose the "best" object.
                float min_depth = numeric_limits<float>::max();
                int best_obj_idx = -1;
                int j = 0;
                for (const auto& object : obj_info.object_pixels) {
                    auto px = object[0];
                    float z = obj_info.cloud->at(px.x, px.y).z;
                    if (z < min_depth) {
                        min_depth = z;
                        best_obj_idx = j;
                    }
                    ++j;
                }

                // Translate pixel coordinates back to original image from ROI.
                Point2i tl_px(obj_info.roi.x, obj_info.roi.y);
                vector<vector<Point2i>> trans_object_px;
                for (const auto& object : obj_info.object_pixels) {
                    trans_object_px.push_back(
                        translate_px_coords(object, tl_px));
                }

                if (show_feeds) {
                    Mat masked = draw_color_on_depth(color_mat, depth_u16_mat);
                    int j = 0;
                    for (const auto& object : trans_object_px) {
                        Vec3b color = j == best_obj_idx ?
                            Vec3b(0, 0, 255) :
                            Vec3b(dis(gen), dis(gen), dis(gen));
                        draw_pixels(masked, object, color);
                        ++j;
                    }
                    imshow("kinect_feed", masked);

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
                    imshow("edges", edges);

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

                    object_grid.update(
                        trans_object_px, object_medoids, edge_medoids);
                    Mat weights = object_grid.get_weights();
                    threshold(weights, weights, 0.9, 0, THRESH_TOZERO);
                    imshow("objects", weights);
                }

                Point2i medoid =
                    region_medoid(obj_info.object_pixels[best_obj_idx]);

                // Send grasping point to the Rexarm.
                auto normal_cloud = estimate_normals(obj_info.cloud);
                GraspingPoint gp;
                gp.point = vec3f_from_pointxyz(
                    obj_info.cloud->at(medoid.x, medoid.y));
                gp.normal = vec3f_from_normal(
                    normal_cloud->at(medoid.x, medoid.y));
                gp.time_ms = depth_frame.getTimestamp();
                sendto(
                    sock,
                    &gp,
                    sizeof(gp),
                    0,
                    (sockaddr*)&rex_addr,
                    sizeof(rex_addr));
            }
        }

        key = waitKey(1);
    }

    color_stream.stop();
    color_stream.destroy();
    depth_stream.stop();
    depth_stream.destroy();
    device.close();
    OpenNI::shutdown();
    return 0;
}
