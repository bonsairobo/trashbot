#include <iostream>
#include "openni_device_listener.hpp"
#include "kinect_receiver.hpp"

using namespace std;
using namespace openni;

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

    // Create Kinect streams and register new-frame callbacks to the receiver.
    VideoStream color;
    if (try_start_video_stream(color, device, SENSOR_COLOR, "color") != 0)
        return 1;
    VideoStream depth;
    if (try_start_video_stream(depth, device, SENSOR_DEPTH, "depth") != 0)
        return 1;
    KinectReceiver recv(color, depth);
    recv.make_windows();
    recv.loop_until_esc();

    // Clean up.
    depth.stop();
    depth.destroy();
    color.stop();
    color.destroy();
    device.close();
    OpenNI::shutdown();
    return 0;
}
