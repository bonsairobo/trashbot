#include "kinect_receiver.hpp"
#include <iostream>

using namespace openni;
using namespace cv;
using namespace std;

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
    ofstream *log_stream,
    bool set_reg)
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
    if (set_reg and device.isImageRegistrationModeSupported(
            IMAGE_REGISTRATION_DEPTH_TO_COLOR))
    {
        device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
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

int KinectReceiver::try_start_streams(Device& device) {
    if (try_start_video_stream(
        color, device, SENSOR_COLOR, "color", log_stream, false) != 0)
    {
        return 1;
    }
    if (try_start_video_stream(
        depth, device, SENSOR_DEPTH, "depth", log_stream, true) != 0)
    {
        return 1;
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

void KinectReceiver::update_model(const Mat& depth, const Mat& color) {
    // TODO: Use cv::rgbd::RgbdOdometry to do visual odometry.

    // TODO: Detect graspable objects.

    // TODO: Find grasping points.

    // TODO: Write object and Rexarm feedback data to socket.
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

        update_model(depth_cpy, color_cpy);
    }
}
