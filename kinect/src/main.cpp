#include "../../common/socket_types.hpp"
#include "img_proc.hpp"
#include "common.hpp"
#include "occupancy_grid.hpp"
#include "trash_search.hpp"
#include <fstream>
#include <random>
#include <fcntl.h>
#include <cassert>

using namespace std;
using namespace openni;
using namespace cv;
using namespace pcl;
using namespace Eigen;

static int closest_object_index(
    const vector<Point2i>& medoids, PointCloud<PointXYZ>::ConstPtr cloud)
{
    if (cloud == nullptr) {
        return -1;
    }

    // Choose the closest object to the Kinect.
    float min_dist = numeric_limits<float>::max();
    int best_obj_idx = -1;
    int obj_idx = 0;
    for (const auto& medoid : medoids) {
        PointXYZ pt = cloud->at(medoid.x, medoid.y);
        float d = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (d < min_dist) {
            min_dist = d;
            best_obj_idx = obj_idx;
        }
        ++obj_idx;
    }
    return best_obj_idx;
}

int main(int argc, char **argv) {
    // Open windows for drawing streams.
    bool show_feeds = !(argc > 1 and *(argv[1]) == '0');

    // Saves all frames from depth and color streams in ONI files.
    bool record_streams = argc > 2 and *(argv[2]) == '1';

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
    }

    // Create RNG for RGB values.
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint8_t> dis(100, 200);

    // Set up UDP socket for receiving command from joystick and sending data
    // to Rexarm.
    sockaddr_un js_addr = create_udp_addr("/tmp/joystick_endpoint");
    sockaddr_un kin_addr = create_udp_addr("/tmp/kinect_endpoint");
    sockaddr_un rex_addr = create_udp_addr("/tmp/rexarm_endpoint");
    sockaddr_un mc_addr = create_udp_addr("/tmp/motion_controller_endpoint");
    int sock = try_create_udp_socket();
    try_bind_path(sock, kin_addr);
    fcntl(sock, F_SETFL, O_NONBLOCK);

    VideoMode vm = depth_stream.getVideoMode();
    OccupancyGrid object_grid(vm.getResolutionX(), vm.getResolutionY());

    bool manual_mode = true;
    bool pickup_complete = false;
    TrashSearch trash_search;

    Point3f search_ftl(-300.0, 250.0, 650.0);
    Point3f search_bbr(300.0, -250.0, 2000.0);
    Rect search_roi = roi_from_workspace_corners(
        search_ftl, search_bbr, depth_stream);
    Point3f pickup_ftl(-200.0, -50.0, 650.0);
    Point3f pickup_bbr(200.0, -250.0, 950.0);
    Rect pickup_roi = roi_from_workspace_corners(
         pickup_ftl, pickup_bbr, depth_stream);

    // Search odds are very large for more responsive object recognition in
    // autonomous mode (when the update rate is slow).
    uint8_t search_hit_odds = 250, search_miss_odds = 100;
    uint8_t pickup_hit_odds = 20, pickup_miss_odds = 5;

    uint8_t key = 0;
    const uint8_t ESC_KEYCODE = 27;
    const uint8_t SPACEBAR = 32;
    while (key != ESC_KEYCODE) {
        // Select workspace based on control mode.
        Point3f ftl = manual_mode ? pickup_ftl : search_ftl;
        Point3f bbr = manual_mode ? pickup_bbr : search_bbr;
        Rect roi = manual_mode ? pickup_roi : search_roi;

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

        // Check for commands to search for grasping points or switch modes.
        CodePacket cmd(NONE_TYPE);
        socklen_t len = sizeof(js_addr);
        ssize_t bytes_read = 1;
        bool do_send_grasp = key == SPACEBAR;
        sockaddr_un addr;
        while (bytes_read > 0) {
            bytes_read = recvfrom(
                sock,
                &cmd,
                sizeof(cmd),
                0,
                (sockaddr*)&addr,
                &len);
            if (bytes_read < 0) {
                if (errno == EAGAIN) {
                    break;
                }
                perror("recvfrom");
                exit(1);
            } else if (strcmp(addr.sun_path, js_addr.sun_path) == 0 and
                bytes_read == sizeof(cmd))
            {
                // Do nothing, we already have the code packet.
            } else if (strcmp(addr.sun_path, rex_addr.sun_path) == 0 and
                bytes_read == 1)
            {
                pickup_complete = true;
                break;
            } else {
                cerr << "Bad packet!" << endl;
                continue;
            }

            // Consume all commands and only do a single search.
            if (manual_mode and cmd.type == PICKUP_COMMAND) {
                do_send_grasp = true;
            } else if (cmd.type == MODE_SWITCH_COMMAND) {
                trash_search = TrashSearch(); // reset state machine
                manual_mode = !manual_mode;
                if (manual_mode) {
                    object_grid.reset();
                    object_grid.set_update_odds(
                        pickup_hit_odds, pickup_miss_odds);
                } else {
                    object_grid.reset();
                    object_grid.set_update_odds(
                        search_hit_odds, search_miss_odds);
                }
                cout << "manual_mode = " << manual_mode << endl;
            }
        }

        StopWatch watch;
        watch.start();

        // Get pixels, cloud, and ROI for workspace objects.
        ObjectInfo obj_info = get_workspace_objects(
            depth_stream, depth_f32_mat, ftl, bbr, roi, 100, 2.0, 4.3);

        cout << "get_workspace_objects took "
             << watch.click() << " seconds." << endl;

        // Merge similar planes (by unit normal dot product and distance).
        PlaneInfo merged_plane_info = merge_similar_planes(obj_info.plane_info);

        cout << "merge_similar_planes took "
             << watch.click() << " seconds." << endl;

        // Translate pixel coordinates back to original image from ROI.
        Point2i tl_px(roi.x, roi.y);
        vector<vector<Point2i>> trans_object_px;
        for (const auto& object : obj_info.object_pixels) {
            trans_object_px.push_back(
                translate_px_coords(object, tl_px));
        }

        cout << "translate_px_coords took "
             << watch.click() << " seconds." << endl;

        // Make edge image.
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

        cout << "making edge image took "
             << watch.click() << " seconds." << endl;

        // Extract objects from edges in point cloud ROI.
        vector<vector<Point2i>> edge_objects =
            find_nonzero_components<uint8_t>(edges);
        remove_small_regions(&edge_objects, 30);

        cout << "find_nonzero_components took "
             << watch.click() << " seconds." << endl;

        // Do object-edge correspondence filtering.
        vector<Point2i> object_medoids;
        for (const auto& obj : trans_object_px) {
            object_medoids.push_back(region_medoid(obj));
        }
        vector<Point2i> edge_medoids;
        for (const auto& obj : edge_objects) {
            edge_medoids.push_back(region_medoid(obj));
        }
        if (!edge_medoids.empty() and !object_medoids.empty()) {
            object_grid.update(
                trans_object_px,
                edge_objects,
                object_medoids,
                edge_medoids);
        }
        Mat weights = object_grid.get_weights();
        threshold(weights, weights, 0.9, 0, THRESH_TOZERO);
        auto final_objects =
            find_nonzero_components<float>(weights);
        remove_small_regions(&final_objects, 100);
        vector<Point2i> final_medoids;
        for (const auto& object : final_objects) {
            final_medoids.push_back(region_medoid(object) - tl_px);
        }

        cout << "entire object filtering took "
             << watch.click() << " seconds." << endl;

        // Choose the closest object to the Kinect.
        int best_obj_idx = closest_object_index(
            final_medoids, obj_info.cloud);

        cout << "choosing closest object took "
             << watch.click() << " seconds." << endl;

        if (show_feeds) {
            Mat masked = mask_image<uint16_t, Vec3b>(
                color_mat, depth_u16_mat);
            int j = 0;
            for (const auto& object : final_objects) {
                Vec3b color = j == best_obj_idx ?
                    Vec3b(0, 0, 255) :
                    Vec3b(dis(gen), dis(gen), dis(gen));
                draw_pixels(masked, object, color);
                ++j;
            }
            imshow("kinect_feed", masked);

            Mat color_edges;
            cvtColor(edges, color_edges, CV_GRAY2BGR);
            //draw_points(color_edges, edge_medoids, Vec3b(0,0,255));
            //draw_points(color_edges, object_medoids, Vec3b(255,0,0));
            imshow("edges", color_edges);

            cout << "drawing feeds took "
                 << watch.click() << " seconds." << endl;
        }

        cout << "# planes = " << merged_plane_info.plane_eqs.size() << endl;
        for (const auto& plane_eq : merged_plane_info.plane_eqs) {
            cout << "Normal equation: ["
                 << plane_eq[0] << ","
                 << plane_eq[1] << ","
                 << plane_eq[2] << ","
                 << plane_eq[3] <<  "]" << endl;
        }

        cout << "# objects = " << final_objects.size() << endl;

        if (!manual_mode and obj_info.cloud != nullptr) {
            // Execute trash search state machine.
            auto medoid = best_obj_idx == -1 ?
                Point2i(0,0) : final_medoids[best_obj_idx];
            if (trash_search.update(
                pickup_complete,
                best_obj_idx >= 0,
                pickup_ftl,
                pickup_bbr,
                merged_plane_info,
                obj_info.cloud->at(medoid.x, medoid.y)))
            {
                do_send_grasp = true;
            }

            // Send motor amplitudes to motion controller.
            cout << "motors: " << trash_search.motors.l_motor << " "
                 << trash_search.motors.r_motor << endl;
            sendto(
                sock,
                &(trash_search.motors),
                sizeof(trash_search.motors),
                0,
                (sockaddr*)&mc_addr,
                sizeof(mc_addr));

            cout << "trash search state machine took "
                 << watch.click() << " seconds." << endl;
        }

        // Send grasping point to the Rexarm.
        if (!final_objects.empty() and do_send_grasp) {
            // Compute the principal axis unit vector of the chosen object.
            Vector3f principal_axis = object_principal_axis(
                translate_px_coords(final_objects[best_obj_idx], -tl_px),
                obj_info.cloud);
            log_stream << "principal axis = (" << principal_axis(0) << ","
                       << principal_axis(1) << "," << principal_axis(2)
                       << ")" << endl;

            Point2i medoid = region_medoid(final_objects[best_obj_idx]);
            medoid -= tl_px;
            auto normal_cloud = estimate_normals(obj_info.cloud);
            GraspingPoint gp;
            gp.point = vec3f_from_pointxyz(
                obj_info.cloud->at(medoid.x, medoid.y));
            gp.normal = vec3f_from_normal(
                normal_cloud->at(medoid.x, medoid.y));
            gp.time_ms = depth_frame.getTimestamp();
            gp.principal_axis = vec3f_from_eigen_vector3f(principal_axis);
            log_stream << "GP = ("
                       << gp.point.x << ","
                       << gp.point.y << ","
                       << gp.point.z << ")" << endl;
            sendto(
                sock,
                &gp,
                sizeof(gp),
                0,
                (sockaddr*)&rex_addr,
                sizeof(rex_addr));
            cout << "sent grasp packet" << endl;
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
