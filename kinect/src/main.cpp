#include "../../common/socket_types.hpp"
#include "grasping_model.hpp"
#include "localization_model.hpp"
#include "img_proc.hpp"
#include "common.hpp"
#include <fstream>
#include <random>

using namespace std;
using namespace openni;
using namespace cv;
using namespace pcl;

static const uint8_t ESC_KEYCODE = 27;
static const int NUM_SAMPLES = 5;

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

    VideoMode vm = depth_stream.getVideoMode();

    Recorder recorder;
    if (record_streams) {
        recorder.create("./rgbd_stream.ONI");
        recorder.attach(color_stream);
        recorder.attach(depth_stream);
        recorder.start();
    }

    // Open webcam.
    VideoCapture webcam(0);
    if (!webcam.isOpened()) {
        cout << "Did not find webcam." << endl;
    }

    if (show_feeds) {
        namedWindow("kinect_feed", 1);
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

    //LocalizationModel loc_model;
    //GraspingModel grasp_model;

    Mat mean_depth_img(
        vm.getResolutionY(), vm.getResolutionX(), CV_32F, Scalar(0.0));
    int n_samp = 0;

    uint8_t key = 0;
    while (key != ESC_KEYCODE) {
        // Block until new frame data is ready.
        Mat depth_u16_mat, color_mat, webcolor_mat;
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
        if (webcam.isOpened())
            webcam.read(webcolor_mat);

        if (record_streams and webcam.isOpened()) {
            // Use depth frame timestamp for relative ordering, but timestamps
            // don't get saved to ONI.
            imwrite(
                "images/webcolor" +
                    to_string(depth_frame.getTimestamp()) + ".png",
                webcolor_mat);
        }

        Mat depth_f32_mat;
        depth_u16_mat.convertTo(depth_f32_mat, CV_32F);
        mean_depth_img += (1.0 / NUM_SAMPLES) * depth_f32_mat;
        ++n_samp;

        // Check for command to search for grasping points.
        /*PickupCommand cmd;
        socklen_t len = sizeof(js_addr);
        ssize_t bytes_read = 1;*/
        bool do_search = n_samp == NUM_SAMPLES;
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
            // Get pixels, points, normals, and ROI for workspace objects.
            ObjectInfo obj_info =
                get_workspace_objects(depth_stream, depth_f32_mat);
            if (!obj_info.object_pixels.empty()) {
                Point2i tl_px(obj_info.roi.x, obj_info.roi.y);

                // TODO: see if normal estimation would benefit from an
                // optimization that cuts the cloud image into sub-images, one
                // for each object.
                auto normal_cloud = estimate_normals(obj_info.cloud);

                // Choose the "best" object.
                // TODO: make this smarter.
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

                if (show_feeds) {
                    Mat masked = draw_color_on_depth(color_mat, depth_u16_mat);
                    int j = 0;
                    for (const auto& object : obj_info.object_pixels) {
                        Vec3b color = j == best_obj_idx ?
                            Vec3b(0, 0, 255) :
                            Vec3b(dis(gen), dis(gen), dis(gen));
                        draw_pixels(
                            masked, translate_px_coords(object, tl_px), color);
                        ++j;
                    }
                    imshow("kinect_feed", masked);
                }

                Point2i centroid =
                    region_centroid(obj_info.object_pixels[best_obj_idx]);

                // Send grasping point to the Rexarm.
                GraspingPoint gp;
                gp.point = vec3f_from_pointxyz(
                    obj_info.cloud->at(centroid.x, centroid.y));
                gp.normal = vec3f_from_normal(
                    normal_cloud->at(centroid.x, centroid.y));
                gp.time_ms = depth_frame.getTimestamp();
                sendto(
                    sock,
                    &gp,
                    sizeof(gp),
                    0,
                    (sockaddr*)&rex_addr,
                    sizeof(rex_addr));
            }

            // Reset sampling counter and mean image.
            n_samp = 0;
            mean_depth_img = Scalar(0.0);
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
