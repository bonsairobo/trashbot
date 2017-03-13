#include <libfreenect.h>
#include <iostream>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include "OpenNI.h"
#include <cstdint>
#include <mutex>
#include <atomic>

using namespace std;
using namespace openni;
using namespace cv;

static bool g_depth_set = false;
static bool g_color_set = false;
static Mat g_depth_img;
static mutex g_depth_mutx;
static Mat g_color_img;
static mutex g_color_mutx;

// Convert OpenNI image format to OpenCV format.
static Mat cv_image_from_vframe_ref(const VideoFrameRef& frame, int n_bytes) {
    int type = n_bytes == 3 ? CV_8UC3 : CV_16UC1;
    Mat out(frame.getHeight(), frame.getWidth(), type);
    memcpy(out.data, frame.getData(),
        frame.getWidth() * frame.getHeight() * n_bytes);
    return out;
}

static void draw_frame(const VideoFrameRef& frame) {
    switch (frame.getVideoMode().getPixelFormat()) {
    case PIXEL_FORMAT_RGB888:
        cout << "NEW COLOR FRAME" << endl;
        g_color_mutx.lock();
        g_color_img = cv_image_from_vframe_ref(frame, 3);
        g_color_set = true;
        g_color_mutx.unlock();
        break;
    case PIXEL_FORMAT_DEPTH_1_MM:
    case PIXEL_FORMAT_DEPTH_100_UM:
        cout << "NEW DEPTH FRAME" << endl;
        g_depth_mutx.lock();
        g_depth_img = cv_image_from_vframe_ref(frame, 2);
        g_depth_set = true;
        g_depth_mutx.unlock();
        break;
    default:
        cout << "Unknown format" << endl;
    }
}

// Event-based frame processing.
class FrameCallback : public VideoStream::NewFrameListener {
public:
    void onNewFrame(VideoStream& stream) {
        stream.readFrame(&m_frame);
        draw_frame(m_frame);
    }
private:
    VideoFrameRef m_frame;
};

class OpenNIDeviceListener : public OpenNI::DeviceConnectedListener,
                             public OpenNI::DeviceDisconnectedListener,
                             public OpenNI::DeviceStateChangedListener
{
public:
    virtual void onDeviceStateChanged(
        const DeviceInfo* pInfo, DeviceState state) 
    {
        cout << "Device " << pInfo->getUri()
             << " error state changed to " << state << endl;
    }

    virtual void onDeviceConnected(const DeviceInfo* pInfo) {
        cout << "Device " << pInfo->getUri() << " connected" << endl;
    }

    virtual void onDeviceDisconnected(const DeviceInfo* pInfo) {
        cout << "Device " << pInfo->getUri() << " disconnected" << endl;
    }
};

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

int main() {
    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK) {
        cout << "Initialize failed" << endl
             << OpenNI::getExtendedError() << endl;
        return 1;
    }

    // Register callbacks for device events.
    OpenNIDeviceListener devicePrinter;
    OpenNI::addDeviceConnectedListener(&devicePrinter);
    OpenNI::addDeviceDisconnectedListener(&devicePrinter);
    OpenNI::addDeviceStateChangedListener(&devicePrinter);

    // Enumerate devices and choose the first one.
    openni::Array<openni::DeviceInfo> deviceList;
    openni::OpenNI::enumerateDevices(&deviceList);
    for (int i = 0; i < deviceList.getSize(); ++i) {
        cout << "Device " << deviceList[i].getUri()
             << " already connected" << endl;
    }
    Device device;
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK) {
        cout << "Couldn't open device" << endl
             << OpenNI::getExtendedError() << endl;
        return 1;
    }

    // Register callbacks for new frame events.
    FrameCallback color_frame_cb, depth_frame_cb;
    VideoStream color;
    if (try_start_video_stream(color, device, SENSOR_COLOR, "color") != 0)
        return 1;
    color.addNewFrameListener(&color_frame_cb);
    VideoStream depth;
    if (try_start_video_stream(depth, device, SENSOR_DEPTH, "depth") != 0)
        return 1;
    depth.addNewFrameListener(&depth_frame_cb);

    namedWindow("kinect_color", 1);
    namedWindow("kinect_depth", 1);

    // Wait for new frames.
    char key = 0;
    while (key != 27) { // escape
        g_depth_mutx.lock();
        if (g_depth_set)
            imshow("kinect_depth", g_depth_img);
        g_depth_mutx.unlock();
        g_color_mutx.lock();
        if (g_color_set)
            imshow("kinect_color", g_color_img);
        g_color_mutx.unlock();
        key = waitKey(30);
    }

    // Clean up.
    depth.removeNewFrameListener(&depth_frame_cb);
    depth.stop();
    depth.destroy();
    color.removeNewFrameListener(&color_frame_cb);
    color.stop();
    color.destroy();
    device.close();
    OpenNI::shutdown();
    return 0;
}
