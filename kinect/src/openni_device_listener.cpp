#include "openni_device_listener.hpp"
#include <iostream>

using namespace std;
using namespace openni;

void OpenNIDeviceListener::onDeviceStateChanged(
    const DeviceInfo* pInfo, DeviceState state) 
{
    cout << "Device " << pInfo->getUri()
         << " error state changed to " << state << endl;
}

void OpenNIDeviceListener::onDeviceConnected(const DeviceInfo* pInfo) {
    cout << "Device " << pInfo->getUri() << " connected" << endl;
}

void OpenNIDeviceListener::onDeviceDisconnected(const DeviceInfo* pInfo) {
    cout << "Device " << pInfo->getUri() << " disconnected" << endl;
}
