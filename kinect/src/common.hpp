#ifndef KINECT_COMMON_HPP
#define KINECT_COMMON_HPP

#include <opencv2/opencv.hpp>
#include <OpenNI.h>
#include <iostream>

struct NIManager {
    openni::Device device;
    openni::VideoStream depth_stream, color_stream;
    openni::Recorder recorder;

    // Create Kinect streams and register new-frame callbacks to the receiver.
    int open(bool record, std::ostream& log_stream);

    ~NIManager();
};

int get_mat_from_stream(
    openni::VideoStream& stream,
    cv::Mat& mat,
    std::ostream& log_stream,
    int byte_depth);

#endif // KINECT_COMMON_HPP
