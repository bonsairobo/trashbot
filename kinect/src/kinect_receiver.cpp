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

KinectReceiver::KinectReceiver(VideoStream& color, VideoStream& depth):
    depth_set(false),
    color_set(false),
    color_cb(this),
    depth_cb(this)
{
    color.addNewFrameListener(&color_cb);
    depth.addNewFrameListener(&depth_cb);
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
