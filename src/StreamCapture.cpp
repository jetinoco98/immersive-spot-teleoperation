#include "StreamCapture.hpp"
#include <iostream>
#include <chrono>

// ============================================================================
//                            Constructor/Destructor
// ============================================================================

StreamCapture::StreamCapture(const std::string& stream_address) {
    // Build the pipeline string
    pipeline_ = "rtspsrc location=" + stream_address + " latency=50 ! "
                "rtph264depay ! h264parse ! decodebin ! videoconvert ! "
                "video/x-raw, format=BGR ! appsink";
}

StreamCapture::~StreamCapture() {
    try {
        stop();
    } catch (const std::exception& e) {
        std::cerr << "Failed to cleanup StreamCapture: " << e.what() << std::endl;
    }
}


// ============================================================================
//                                 Public Methods
// ============================================================================

bool StreamCapture::start() {
    cv_capture_.open(pipeline_, cv::CAP_GSTREAMER);
    if (!cv_capture_.isOpened()) {
        std::cerr << "Error: Unable to open video stream." << std::endl;
        return false;
    }

    buffer_.run = true;
    buffer_.new_frame = true;
    capture_thread_ = std::thread(&StreamCapture::captureLoop, this);
    return true;
}

int StreamCapture::getWidth() {
    return static_cast<int>(cv_capture_.get(cv::CAP_PROP_FRAME_WIDTH));
}

int StreamCapture::getHeight() {
    return static_cast<int>(cv_capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
}


// ============================================================================
//                                Private Methods
// ============================================================================

void StreamCapture::captureLoop() {
    cv::Mat frame;
    auto last_success = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(5); // Customize the timeout

    // Loop while the main loop is not over
    while (buffer_.run) {
        // try to grab a new image
        if (cv_capture_.read(frame)) {
            last_success = std::chrono::steady_clock::now(); // Reset the timer

            // Copy both left and right images
            buffer_.mtx.lock();
            // Define the first ROI (left part of the frame)
            cv::Rect roi1(0, 0, frame.cols / 2, frame.rows);
            buffer_.leftImage = frame(roi1).clone(); // Use clone() to copy the data

            // Define the second ROI (right part of the frame)
            cv::Rect roi2(frame.cols / 2, 0, frame.cols / 2, frame.rows);
            buffer_.rightImage = frame(roi2).clone(); // Use clone() to copy the data

            buffer_.mtx.unlock();
            buffer_.new_frame = true;
        }
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(2)); // DEBUG 
        // Check for timeout
        auto now = std::chrono::steady_clock::now();
        if (now - last_success > timeout) {
            std::cerr << "[ERROR] No frame received for 5 seconds. Stopping capture thread..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
            std::exit(EXIT_FAILURE);
        }
    }
}
    
void StreamCapture::stop() {
    if (buffer_.run) {
        buffer_.run = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        cv_capture_.release();
    }
    cv::destroyAllWindows();
}