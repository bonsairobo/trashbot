#ifndef KINECT_COMMON_HPP
#define KINECT_COMMON_HPP

#include <opencv2/opencv.hpp>
#include <OpenNI.h>
#include <iostream>
#include <chrono>

class StopWatch {
    std::chrono::high_resolution_clock::time_point prev_time;
public:
    void start();
    float click();
};

int try_start_rgbd_streams(
    openni::Device&,
    openni::VideoStream& depth_stream,
    openni::VideoStream& color_stream,
    std::ostream& log_stream,
    bool do_registration);

int get_mat_from_stream(
    openni::VideoStream& stream,
    cv::Mat& mat,
    std::ostream& log_stream,
    int n_bytes,
    openni::VideoFrameRef *frame_out);

#endif // KINECT_COMMON_HPP
