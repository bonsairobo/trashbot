#include <libfreenect.h>
#include <iostream>
#include <signal.h>
#include "OpenNI.h"

using namespace std;
using namespace openni;

void analyzeFrame(const VideoFrameRef& frame) {
    DepthPixel* pDepth;
    RGB888Pixel* pColor;

    int middleIndex = (frame.getHeight()+1)*frame.getWidth()/2;

    switch (frame.getVideoMode().getPixelFormat()) {
    case PIXEL_FORMAT_DEPTH_1_MM:
    case PIXEL_FORMAT_DEPTH_100_UM:
        pDepth = (DepthPixel*)frame.getData();
        printf("[%08llu] %8d\n",
            (long long)frame.getTimestamp(),
            pDepth[middleIndex]);
        break;
    case PIXEL_FORMAT_RGB888:
        pColor = (RGB888Pixel*)frame.getData();
        printf("[%08llu] 0x%02x%02x%02x\n",
            (long long)frame.getTimestamp(),
            pColor[middleIndex].r&0xff,
            pColor[middleIndex].g&0xff,
            pColor[middleIndex].b&0xff);
        break;
    default:
        cout << "Unknown format" << endl;
    }
}

class FrameCallback : public VideoStream::NewFrameListener {
public:
    void onNewFrame(VideoStream& stream) {
        stream.readFrame(&m_frame);
        analyzeFrame(m_frame);
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

static void sleep(int millisecs) {
    usleep(millisecs * 1000);
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

    // Wait for new frames.
    while (true) {
        sleep(100);
    }

    depth.removeNewFrameListener(&frame_cb);
    depth.stop();
    depth.destroy();
    device.close();
    OpenNI::shutdown();

    return 0;
}
