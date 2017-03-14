#ifndef KINECT_RECEIVER_HPP
#define KINECT_RECEIVER_HPP

#include <mutex>
#include <opencv2/opencv.hpp>
#include <OpenNI.h>
#include "frame_callback.hpp"
#include <fstream>

class KinectReceiver {
    friend FrameCallback;

    const bool show_feeds;
    bool depth_set;
    bool color_set;
    cv::Mat depth_img;
    std::mutex depth_mutx;
    cv::Mat color_img;
    std::mutex color_mutx;
    FrameCallback color_cb, depth_cb;
    openni::VideoStream depth, color;
    std::ofstream *log_stream;

    void update();
    void update_model(const cv::Mat& depth, const cv::Mat& color);
    void write_mat(const openni::VideoFrameRef&);

public:
    KinectReceiver(bool show_feeds, std::ofstream *log_stream);
    void maybe_make_windows() const;
    void loop_until_esc();
    void close_streams();
    int try_start_streams(openni::Device&);
};

#endif // KINECT_RECEIVER_HPP
