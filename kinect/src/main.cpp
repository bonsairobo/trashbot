#include "../../common/socket_types.hpp"
#include "grasping_model.hpp"
#include "localization_model.hpp"
#include "img_proc.hpp"
#include "common.hpp"
#include <fstream>

using namespace std;
using namespace openni;
using namespace cv;

int main(int argc, char **argv) {
    // Open windows for drawing streams realtime.
    bool show_feeds = !(argc > 1 and *(argv[1]) == '0');

    // Saves all frames from depth and color streams in ONI files.
    bool record_streams = argc > 2 and *(argv[2]) == '1';

    // All debug and error messages get saved. `watch` the file for
    // realtime feedback.
    ofstream log_stream("kinect_log.txt");

    NIManager ni_man;
    ni_man.open(record_streams, log_stream);

    // Set up UDP socket for receiving command from joystick and sending data
    // to Rexarm.
    sockaddr_un js_addr = create_udp_addr("/tmp/joystick_endpoint");
    sockaddr_un kin_addr = create_udp_addr("/tmp/kinect_endpoint");
    sockaddr_un rex_addr = create_udp_addr("/tmp/rexarm_endpoint");
    int sock = try_create_udp_socket();
    try_bind_path(sock, kin_addr);

    if (show_feeds) {
        namedWindow("masked color", 1);
    }

    LocalizationModel loc_model;
    GraspingModel grasp_model;

    char key = 0;
    while (key != 27) { // escape
        // Block until new frame data is ready.
        Mat depth_mat, color_mat;
        if (get_mat_from_stream(
            ni_man.depth_stream, depth_mat, log_stream, 2) != 0)
        {
            return 1;
        }
        if (get_mat_from_stream(
            ni_man.color_stream, color_mat, log_stream, 3) != 0)
        {
            return 1;
        }

        if (show_feeds) {
            Mat masked = draw_color_on_depth(color_mat, depth_mat);
            imshow("masked color", masked);
        }

        loc_model.update(depth_mat, color_mat);

        // Check for command to search for grasping points.
        PickupCommand cmd;
        socklen_t len = sizeof(js_addr);
        ssize_t bytes_read = 1;
        bool do_search = false;
        while (bytes_read > 0) {
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
        }

        if (do_search) {
            GraspingPoints points = grasp_model.search_grasping_points(
                depth_mat, color_mat);
            sendto(
                sock,
                &points,
                sizeof(points),
                0,
                (sockaddr*)&rex_addr,
                sizeof(rex_addr));
        }
        key = waitKey(5);
    }

    return 0;
}
