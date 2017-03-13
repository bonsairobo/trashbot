#include <libfreenect.h>
#include <iostream>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include "OpenNI.h"
#include <cstdint>

using namespace std;
using namespace openni;
using namespace cv;

// zero-based rectangular frame indexing
template<typename T>
static const T* get_pixel(const VideoFrameRef& frame, int x, int y, int n_bytes)
{
    return ((T*)frame.getData() + y * (frame.getStrideInBytes() / n_bytes) + x);
}

static Mat cv_image_from_vframe_ref(const VideoFrameRef& frame, int n_bytes) {
    int type = n_bytes == 3 ? CV_8UC3 : CV_16UC1;
    Mat out(frame.getHeight(), frame.getWidth(), type);
    if (type == CV_8UC3) {
        for (int y = 0; y < frame.getHeight(); ++y) {
            for (int x = 0; x < frame.getWidth(); ++x) {
                const RGB888Pixel *px =
                    get_pixel<RGB888Pixel>(frame, x, y, n_bytes);
                out.at<Vec3b>(x,y) = Vec3b(px->r, px->g, px->b);
            }
        }
    } else if (type == CV_16UC1) {
        for (int y = 0; y < frame.getHeight(); ++y) {
            for (int x = 0; x < frame.getWidth(); ++x) {
                const DepthPixel *px =
                    get_pixel<DepthPixel>(frame, x, y, n_bytes);
                out.at<uint16_t>(x,y) = *px;
            }
        }
    }

    return out;
}

static void draw_frame(const VideoFrameRef& frame) {
    Mat image;

    switch (frame.getVideoMode().getPixelFormat()) {
    case PIXEL_FORMAT_DEPTH_1_MM:
    case PIXEL_FORMAT_DEPTH_100_UM:
        image = cv_image_from_vframe_ref(frame, 1);
        imshow("kinect_depth", image);
        break;
    case PIXEL_FORMAT_RGB888:
        image = cv_image_from_vframe_ref(frame, 3);
        imshow("kinect_color", image);
        break;
    default:
        cout << "Unknown format" << endl;
    }
}

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
        return 2;
    }

    // Register callbacks for new frame events.
    FrameCallback frame_cb;
    VideoStream depth;
    if (try_start_video_stream(depth, device, SENSOR_DEPTH, "depth") != 0)
        return 1;
    depth.addNewFrameListener(&frame_cb);
    VideoStream color;
    if (try_start_video_stream(color, device, SENSOR_COLOR, "color") != 0)
        return 1;
    color.addNewFrameListener(&frame_cb);

    namedWindow("kinect_color", 1);
    namedWindow("kinect_depth", 1);

    // Wait for new frames.
    char key = 0;
    while (key != 27) { // escape
        key = waitKey(10);
    }

    depth.removeNewFrameListener(&frame_cb);
    depth.stop();
    depth.destroy();
    color.removeNewFrameListener(&frame_cb);
    color.stop();
    color.destroy();
    device.close();
    OpenNI::shutdown();

    return 0;
}
