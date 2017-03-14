#ifndef FRAME_CALLBACK_HPP
#define FRAME_CALLBACK_HPP

#include <OpenNI.h>

class KinectReceiver;

class FrameCallback : public openni::VideoStream::NewFrameListener {
    openni::VideoFrameRef frame;
    KinectReceiver *recv_ptr;

public:
    FrameCallback(KinectReceiver *);
    void onNewFrame(openni::VideoStream& stream);
};

#endif // FRAME_CALLBACK_HPP
