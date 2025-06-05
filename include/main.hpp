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
void runPythonScript(const std::string& relativeScriptPath, const std::string& args = "");
RPY quaternionToRPY(const ovrTrackingState& ts);
void processOculusInput(float* data, ovrSession& session, ovrInputState& LastInputState);
AppConfig LoadAppConfig(const std::string& filename = "config.json");
