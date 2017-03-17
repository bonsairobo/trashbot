#include "kinect_receiver.hpp"
#include <iostream>

using namespace openni;
using namespace cv;
using namespace std;


static GraspingPoints search_grasping_points(
    const Mat& depth, const Mat& color)
{
    GraspingPoints points;
    return points;
}

// Convert OpenNI image format to OpenCV format.
static Mat cv_image_from_vframe_ref(const VideoFrameRef& frame, int n_bytes) {
    int type = n_bytes == 3 ? CV_8UC3 : CV_16UC1;
    Mat out(frame.getHeight(), frame.getWidth(), type);
    memcpy(out.data, frame.getData(),
        frame.getWidth() * frame.getHeight() * n_bytes);
    if (type == CV_8UC3)
        cvtColor(out, out, CV_RGB2BGR);
    return out;
}

static int try_start_video_stream(
    VideoStream& stream,
    Device& device,
    SensorType type,
    const string& type_str,
    ofstream *log_stream)
{
    Status rc;
    if (device.getSensorInfo(type) != NULL) {
        rc = stream.create(device, type);
        if (rc != STATUS_OK) {
            *log_stream << "ERROR: Couldn't create " << type_str
                        << " stream" << endl
                        << OpenNI::getExtendedError() << endl;
            return 1;
        }
    }
    rc = stream.start();
    if (rc != STATUS_OK) {
        *log_stream << "ERROR: Couldn't start the " << type_str
                    << " stream" << endl
                    << OpenNI::getExtendedError() << endl;
        return 1;
    }
    return 0;
}

KinectReceiver::KinectReceiver(bool show_feeds, ofstream *log_stream):
    show_feeds(show_feeds),
    depth_set(false),
    color_set(false),
    color_cb(this),
    depth_cb(this),
    log_stream(log_stream)
{}

void KinectReceiver::bind_socket() {
    // Set up UDP socket for receiving command from joystick and sending data
    // to Rexarm.
    js_addr = create_udp_addr("/tmp/joystick_endpoint");
    kin_addr = create_udp_addr("/tmp/kinect_endpoint");
    rex_addr = create_udp_addr("/tmp/rexarm_endpoint");
    sock = try_create_udp_socket();
    try_bind_path(sock, kin_addr);
}

int KinectReceiver::try_start_streams(Device& device) {
    if (try_start_video_stream(
        color, device, SENSOR_COLOR, "color", log_stream) != 0)
    {
        return 1;
    }
    if (try_start_video_stream(
        depth, device, SENSOR_DEPTH, "depth", log_stream) != 0)
    {
        return 1;
    }
    if (device.isImageRegistrationModeSupported(
            IMAGE_REGISTRATION_DEPTH_TO_COLOR))
    {
        device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    }

    color.addNewFrameListener(&color_cb);
    depth.addNewFrameListener(&depth_cb);
    return 0;
}

void KinectReceiver::close_streams() {
    depth.removeNewFrameListener(&depth_cb);
    depth.stop();
    depth.destroy();
    color.removeNewFrameListener(&color_cb);
    color.stop();
    color.destroy();
}

void KinectReceiver::maybe_make_windows() const {
    if (show_feeds) {
        namedWindow("kinect_color", 1);
        namedWindow("kinect_depth", 1);
    }
}

void KinectReceiver::loop_until_esc() {
    char key = 0;
    while (key != 27) { // escape
        update();
        key = waitKey(5);
    }
}

void KinectReceiver::write_mat(const VideoFrameRef& frame) {
    switch (frame.getVideoMode().getPixelFormat()) {
    case PIXEL_FORMAT_RGB888:
        color_mutx.lock();
        color_img = cv_image_from_vframe_ref(frame, 3);
        color_set = true;
        color_mutx.unlock();
        break;
    case PIXEL_FORMAT_DEPTH_1_MM:
    case PIXEL_FORMAT_DEPTH_100_UM:
        depth_mutx.lock();
        depth_img = cv_image_from_vframe_ref(frame, 2);
        depth_set = true;
        depth_mutx.unlock();
        break;
    default:
        *log_stream << "WARNING: Unknown format" << endl;
    }
}

void KinectReceiver::update() {
    // Fetch current images.
    bool images_ready = true;
    Mat depth_cpy, color_cpy;
    depth_mutx.lock();
    images_ready &= depth_set;
    depth_cpy = depth_img;
    depth_mutx.unlock();
    color_mutx.lock();
    images_ready &= color_set;
    color_cpy = color_img;
    color_mutx.unlock();

    if (images_ready) {
        if (show_feeds) {
            imshow("kinect_depth", depth_cpy);
            imshow("kinect_color", color_cpy);
        }

        loc_model.update(depth_cpy, color_cpy);
    }

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
            perror("recvfrom");
            exit(1);
        } else if (bytes_read != sizeof(cmd)) {
            continue;
        }

        // Consume all commands and only do a single search.
        do_search = true;
    }

    if (do_search) {
        GraspingPoints points = search_grasping_points(depth_cpy, color_cpy);
        sendto(
            sock,
            &points,
            sizeof(points),
            0,
            (sockaddr*)&rex_addr,
            sizeof(rex_addr));
    }
}
