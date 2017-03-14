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
    return out;
}

static int try_start_video_stream(
    VideoStream& stream,
    Device& device,
    SensorType type,
    const string& type_str)
{
    Status rc;
    if (device.getSensorInfo(type) != NULL) {
        rc = stream.create(device, type);
        if (rc != STATUS_OK) {
            cout << "Couldn't create " << type_str <<  " stream" << endl
                 << OpenNI::getExtendedError() << endl;
            return 1;
        }
    }
    rc = stream.start();
    if (rc != STATUS_OK) {
        cout << "Couldn't start the " << type_str << " stream" << endl
             << OpenNI::getExtendedError() << endl;
        return 1;
    }
    return 0;
}

KinectReceiver::KinectReceiver():
    depth_set(false),
    color_set(false),
    color_cb(this),
    depth_cb(this)
{}

int KinectReceiver::try_start_streams(Device& device) {
    if (try_start_video_stream(color, device, SENSOR_COLOR, "color") != 0)
        return 1;
    if (try_start_video_stream(depth, device, SENSOR_DEPTH, "depth") != 0)
        return 1;
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

void KinectReceiver::make_windows() const {
    namedWindow("kinect_color", 1);
    namedWindow("kinect_depth", 1);
}

void KinectReceiver::loop_until_esc() {
    // Draw frames to CV window.
    char key = 0;
    while (key != 27) { // escape
        draw_feeds();
        key = waitKey(30);
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
        cout << "Unknown format" << endl;
    }
}

void KinectReceiver::draw_feeds() {
    depth_mutx.lock();
    if (depth_set)
        imshow("kinect_depth", depth_img);
    depth_mutx.unlock();
    color_mutx.lock();
    if (color_set)
        imshow("kinect_color", color_img);
    color_mutx.unlock();
}