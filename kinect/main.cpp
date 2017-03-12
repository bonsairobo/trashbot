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
        printf("[%08llu] %8d\n", (long long)frame.getTimestamp(),
            pDepth[middleIndex]);
        break;
    case PIXEL_FORMAT_RGB888:
        pColor = (RGB888Pixel*)frame.getData();
        printf("[%08llu] 0x%02x%02x%02x\n", (long long)frame.getTimestamp(),
            pColor[middleIndex].r&0xff,
            pColor[middleIndex].g&0xff,
            pColor[middleIndex].b&0xff);
        break;
    default:
        printf("Unknown format\n");
    }
}

class PrintCallback : public VideoStream::NewFrameListener {
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

static void Sleep(int millisecs) {
    usleep(millisecs * 1000);
}

int main() {
    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK) {
        cout << "Initialize failed" << endl
             << OpenNI::getExtendedError() << endl;
        return 1;
    }

    OpenNIDeviceListener devicePrinter;

    OpenNI::addDeviceConnectedListener(&devicePrinter);
    OpenNI::addDeviceDisconnectedListener(&devicePrinter);
    OpenNI::addDeviceStateChangedListener(&devicePrinter);

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

    VideoStream depth;

    if (device.getSensorInfo(SENSOR_DEPTH) != NULL) {
        rc = depth.create(device, SENSOR_DEPTH);
        if (rc != STATUS_OK)
        {
            cout << "Couldn't create depth stream" << endl
                 << OpenNI::getExtendedError() << endl;
        }
    }
    rc = depth.start();
    if (rc != STATUS_OK) {
        cout << "Couldn't start the depth stream" << endl
             << OpenNI::getExtendedError() << endl;
    }

    PrintCallback depthPrinter;

    // Register to new frame
    depth.addNewFrameListener(&depthPrinter);

    // Wait while we're getting frames through the printer
    while (true) {
        Sleep(100);
    }

    depth.removeNewFrameListener(&depthPrinter);

    depth.stop();
    depth.destroy();
    device.close();
    OpenNI::shutdown();

    return 0;
}
