#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <mutex>


struct VideoCaptureFrameBuffer {
    std::mutex mtx;
    cv::Mat leftImage;
    cv::Mat rightImage;
    bool run = false;
    bool new_frame = false;
};

class StreamCapture {
public:
    StreamCapture(const std::string& stream_address);
    ~StreamCapture();

    bool start();
    void stop();

    int width = 0;
    int height = 0;

    VideoCaptureFrameBuffer buffer_;

private:
    void captureLoop();

    std::string pipeline_;
    cv::VideoCapture cv_capture_;
    std::thread capture_thread_;
};