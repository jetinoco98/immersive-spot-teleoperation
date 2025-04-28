#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include <zmq.hpp>

#include <GL/glew.h>

#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <SDL_syswm.h>

#include <Extras/OVR_Math.h>
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

#include <sl/Camera.hpp>

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include <opencv2/opencv.hpp>


int zmq_test() {
    float data[] = { 0.0f, -2.0f, -3.0f, 4.0f, 5.0f, 6.0f };

    zmq::context_t zmq_context(1);
    zmq::socket_t publisher(zmq_context, zmq::socket_type::pub);
    publisher.bind("tcp://*:5555"); 
    std::string topic = "from_cpp";  

    while (true) {
        zmq::message_t topic_msg(topic.data(), topic.size());
        publisher.send(topic_msg, zmq::send_flags::sndmore);

        // Increment first value
        data[0] += 1.0f;

        zmq::message_t data_msg(sizeof(data)); 
        std::memcpy(data_msg.data(), data, sizeof(data));
        publisher.send(data_msg, zmq::send_flags::none);

        printf("Sent data: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\n", 
               data[0], data[1], data[2], data[3], data[4], data[5]);

        // Send every second
        std::this_thread::sleep_for(std::chrono::seconds(1));  
    }
    
    return 0;
}


int glew_test() {
    std::cout << "GLEW version: " << glewGetString(GLEW_VERSION) << std::endl;
    return 0;
}


int sdl2_test() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL failed to init: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("SDL Test", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 640, 480, 0);
    if (!window) {
        std::cerr << "Failed to create window: " << SDL_GetError() << std::endl;
        return 1;
    }

    std::cout << "SDL window created successfully!" << std::endl;

    SDL_Delay(2000); // wait 2 seconds

    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

int ovr_test() {
    ovrResult result = ovr_Initialize(nullptr);
    if (OVR_FAILURE(result)) {
        std::cerr << "Failed to initialize Oculus SDK." << std::endl;
        return -1;
    }

    std::cout << "Oculus SDK initialized successfully." << std::endl;

    ovr_Shutdown();
    return 0;
}

int zed_test() {
    sl::Camera zed;
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 30;

    sl::ERROR_CODE err = zed.open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Failed to open ZED camera: " << sl::toString(err) << std::endl;
        return -1;
    }

    std::cout << "ZED camera opened successfully!" << std::endl;

    zed.close();
    return 0;
}

void cuda_test() {
    int count = 0;
    cudaError_t err = cudaGetDeviceCount(&count);
    if (err != cudaSuccess) {
        std::cerr << "CUDA error: " << cudaGetErrorString(err) << std::endl;
        return;
    }
    std::cout << "CUDA devices available: " << count << std::endl;
}

int opencv_test() {
    cv::Mat img = cv::Mat::zeros(300, 300, CV_8UC3);
    cv::putText(img, "Hello OpenCV!", cv::Point(30, 150),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);

    cv::imshow("Test Window", img);
    cv::waitKey(0);
    return 0;
}


int main(int argc, char* argv[]) {
    opencv_test();
    return 0;
}

