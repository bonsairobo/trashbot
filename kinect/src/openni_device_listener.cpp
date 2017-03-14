#include "openni_device_listener.hpp"
#include <iostream>

using namespace std;
using namespace openni;

OpenNIDeviceListener::OpenNIDeviceListener(ofstream* log_stream):
	log_stream(log_stream) {}

void OpenNIDeviceListener::onDeviceStateChanged(
    const DeviceInfo* pInfo, DeviceState state)
{
    *log_stream << "Device " << pInfo->getUri()
         		<< " error state changed to " << state << endl;
}

void OpenNIDeviceListener::onDeviceConnected(const DeviceInfo* pInfo) {
    *log_stream << "Device " << pInfo->getUri() << " connected" << endl;
}

void OpenNIDeviceListener::onDeviceDisconnected(const DeviceInfo* pInfo) {
    *log_stream << "Device " << pInfo->getUri() << " disconnected" << endl;
}
