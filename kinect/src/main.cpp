#include <iostream>
#include <fstream>
#include "openni_device_listener.hpp"
#include "kinect_receiver.hpp"

using namespace std;
using namespace openni;

int main(int argc, char **argv) {
    bool show_feeds = !(argc > 1 and *(argv[1]) == '0');

    // All debug and error messages get saved. `watch` the file for
    // realtime feedback.
    ofstream log_stream("kinect_log.txt");

    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK) {
        log_stream << "Initialize failed" << endl
                   << OpenNI::getExtendedError() << endl;
        return 1;
    }

    // Register callbacks for device events.
    OpenNIDeviceListener device_printer(&log_stream);
    OpenNI::addDeviceConnectedListener(&device_printer);
    OpenNI::addDeviceDisconnectedListener(&device_printer);
    OpenNI::addDeviceStateChangedListener(&device_printer);

    // Enumerate devices and choose the first one.
    openni::Array<openni::DeviceInfo> device_list;
    openni::OpenNI::enumerateDevices(&device_list);
    for (int i = 0; i < device_list.getSize(); ++i) {
        log_stream << "LOG: Device " << device_list[i].getUri()
                   << " already connected" << endl;
    }
    Device device;
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK) {
        log_stream << "Couldn't open device" << endl
                   << OpenNI::getExtendedError() << endl;
        return 1;
    }

    // Create Kinect streams and register new-frame callbacks to the receiver.
    KinectReceiver recv(show_feeds, &log_stream);
    if (recv.try_start_streams(device) != 0)
        return 1;
    recv.maybe_make_windows();
    recv.loop_until_esc();

    // Clean up.
    recv.close_streams();
    device.close();
    OpenNI::shutdown();
    return 0;
}
