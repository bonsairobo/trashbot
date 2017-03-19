#ifndef KINECT_COMMON_HPP
#define KINECT_COMMON_HPP

#include <opencv2/opencv.hpp>
#include <OpenNI.h>
#include <iostream>

int try_start_rgbd_streams(
    openni::Device&,
    openni::VideoStream& depth_stream,
    openni::VideoStream& color_stream,
    std::ostream& log_stream);

int get_mat_from_stream(
    openni::VideoStream& stream,
    cv::Mat& mat,
    std::ostream& log_stream,
    int n_bytes,
    openni::VideoFrameRef **frame_out);

#endif // KINECT_COMMON_HPP
