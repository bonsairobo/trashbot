#ifndef OPENNI_DEVICE_LISTENER_HPP
#define OPENNI_DEVICE_LISTENER_HPP

#include <OpenNI.h>
#include <fstream>

class OpenNIDeviceListener : public openni::OpenNI::DeviceConnectedListener,
                             public openni::OpenNI::DeviceDisconnectedListener,
                             public openni::OpenNI::DeviceStateChangedListener
{
	std::ofstream *log_stream;

public:
	OpenNIDeviceListener(std::ofstream*);
    virtual void onDeviceStateChanged(
        const openni::DeviceInfo*, openni::DeviceState);
    virtual void onDeviceConnected(const openni::DeviceInfo*);
    virtual void onDeviceDisconnected(const openni::DeviceInfo*);
};

#endif // OPENNI_DEVICE_LISTENER_HPP
