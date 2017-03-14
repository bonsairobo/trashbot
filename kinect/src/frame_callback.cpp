#include "frame_callback.hpp"
#include "kinect_receiver.hpp"

using namespace openni;

FrameCallback::FrameCallback(KinectReceiver *recv_ptr): recv_ptr(recv_ptr) {}

void FrameCallback::onNewFrame(VideoStream& stream) {
    stream.readFrame(&frame);
    recv_ptr->write_mat(frame);
}
