#pragma once

#define NOMINMAX
#include <Windows.h>

// Standard Libraries
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <cmath>
#include <stddef.h>
#include <fstream>
#include <stdexcept>

// ZeroMQ: for inter-process communication
#include <zmq.hpp>

// JSON: for configuration file parsing
#include "json.hpp"
using json = nlohmann::json;

// Local include files
#include "StreamCapture.hpp"
#include "OculusRenderer.hpp"


struct RPY {
    float roll, pitch, yaw;
};

struct AppConfig {
    bool use_katvr;
    std::string ip_address;
    std::string stream_address;
};

// Function declarations
bool InitOculus(ovrSession& session);
void runPythonScript(const std::string& relativeScriptPath, const std::string& args = "");
RPY quaternionToRPY(const ovrTrackingState& ts);
void getOculusInput(float* data, ovrSession& session);
AppConfig LoadAppConfig(const std::string& filename = "config.json");

// Class declarations
class ZMQPublisher {
public:
    ZMQPublisher(const std::string& address);
    void send(const std::string& topic, const void* data, size_t size);
    ~ZMQPublisher();

private:
    zmq::context_t context_;
    zmq::socket_t publisher_;
};