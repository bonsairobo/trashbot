#ifndef KINECT_RECEIVER_HPP
#define KINECT_RECEIVER_HPP

#include <mutex>
#include <opencv2/opencv.hpp>
#include <OpenNI.h>
#include "frame_callback.hpp"

class KinectReceiver {
	friend FrameCallback;

    bool depth_set;
    bool color_set;
    cv::Mat depth_img;
    std::mutex depth_mutx;
    cv::Mat color_img;
    std::mutex color_mutx;
    FrameCallback color_cb, depth_cb;

    void draw_feeds();
    void write_mat(const openni::VideoFrameRef&);

public:
    KinectReceiver(openni::VideoStream& color, openni::VideoStream& depth);
    void make_windows() const;
    void loop_until_esc();
};

#endif // KINECT_RECEIVER_HPP
