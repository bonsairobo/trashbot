#include <iostream>
#include "openni_device_listener.hpp"
#include "kinect_receiver.hpp"

using namespace std;
using namespace openni;

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
    KinectReceiver recv;
    recv.try_start_streams(device);
    recv.make_windows();
    recv.loop_until_esc();

    // Clean up.
    recv.close_streams();
    device.close();
    OpenNI::shutdown();
    return 0;
}
