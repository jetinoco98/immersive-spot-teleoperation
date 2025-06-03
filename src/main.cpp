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


// ============================================================================
//                                 MAIN FUNCTION
// ============================================================================

int main(int argc, char* argv[]) 
{
    // Loads the configuration file (config.json)
    AppConfig config = LoadAppConfig();
    std::string stream_address = config.stream_address;
    std::string ip_address = config.ip_address;

    // Initialize Oculus SDK
    ovrResult result = ovr_Initialize(nullptr);
    if (OVR_FAILURE(result)) {
        std::cerr << "Failed to initialize Oculus SDK" << std::endl;
        return -1;
    }

    // Create Oculus session
    ovrSession oculus_session = nullptr;
    ovrGraphicsLuid luid;
    result = ovr_Create(&oculus_session, &luid);
    if (OVR_FAILURE(result)) {
        std::cerr << "Failed to create Oculus session" << std::endl;
        ovr_Shutdown();
        return -1;
    }

    // Create the StreamCapture object
    StreamCapture stream(stream_address);
    // Start capture of the video stream
    if (!stream.start()) {
        std::cerr << "[Error] Unable to start video stream." << std::endl;
        return -1;
    }

    // Create the OculusRenderer object
    OculusRenderer renderer(oculus_session);
    // Initialize the Oculus renderer
    if (!renderer.initialize(stream.getWidth(), stream.getHeight())) {
        std::cerr << "[Error] Unable to initialize Oculus renderer." << std::endl;
        return -1;
    }

    // Execute python scripts
    runPythonScript("\\..\\..\\scripts\\oculus\\oculus_client.py", ip_address);
    if (config.use_katvr) {
        runPythonScript("\\..\\..\\scripts\\katvr\\katvr_main.py");
    }

	// Initialize ZMQ Socket
    zmq::context_t zmq_context(1);
    zmq::socket_t publisher(zmq_context, zmq::socket_type::pub);
    publisher.connect("tcp://localhost:5555");

    // Create variable for HDM State 
    ovrInputState LastInputState = {};
    float data[9];  


    // ============================================================================
    //                                 MAIN LOOP
    // ============================================================================

    while (true) {

        // --- Update the Oculus renderer
        if (!renderer.update(stream.buffer_)) {
            break;
        }

        // --- Query the HMD for the input state (buttons, thumbsticks, etc.)
        processOculusInput(data, oculus_session, LastInputState);

        // --- Send HDM data over ZeroMQ socket
        // Send the first part of the message (topic)
        std::string topic = "from_hdm"; 
        zmq::message_t topic_msg(topic.data(), topic.size());
        publisher.send(topic_msg, zmq::send_flags::sndmore);

        // Send the second part of the message (HDM data)
        zmq::message_t data_msg(sizeof(data)); 
        std::memcpy(data_msg.data(), data, sizeof(data));
        publisher.send(data_msg, zmq::send_flags::none);
    }

    // CLOSING AND CLEANUP

    // Closing ZeroMQ socket
    publisher.close();
    zmq_context.shutdown();
    zmq_context.close();

    // Cleanup the renderer and the stream
    renderer.shutdown();
    stream.stop();

    // Quit
    return 0;
}


// ===================================================
// ----- FUNCTION DEFINITIONS -----
// ===================================================

void runPythonScript(const std::string& relativeScriptPath, const std::string& args) {
    char currentDir[MAX_PATH];
    GetCurrentDirectoryA(MAX_PATH, currentDir);

    std::string command = "start cmd /k python \""
                        + std::string(currentDir)
                        + relativeScriptPath
                        + "\" " + args;

    if (system(command.c_str()) != 0) {
        throw std::runtime_error("Error executing Python script: " + relativeScriptPath);
    }
}


RPY quaternionToRPY(const ovrTrackingState& ts) {
    RPY rpy;
    float w = ts.HeadPose.ThePose.Orientation.w;
    float x = ts.HeadPose.ThePose.Orientation.x;
    float y = ts.HeadPose.ThePose.Orientation.y;
    float z = ts.HeadPose.ThePose.Orientation.z;

    // Pitch (x-axis rotation)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    rpy.pitch = std::atan2(sinr_cosp, cosr_cosp);

    // Yaw (y-axis rotation)
    float siny_cosp = 2 * (w * y + z * x);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    rpy.yaw = std::atan2(siny_cosp, cosy_cosp);

    // Roll (z-axis rotation)
    float sinp = 2 * (w * z - x * y);
    if (std::abs(sinp) >= 1)
        rpy.roll = std::copysign(M_PI / 2, sinp);
    else
        rpy.roll = std::asin(sinp);

    return rpy;
}


void processOculusInput(float* data, ovrSession& session, ovrInputState& LastInputState) {
    ovrInputState InputState;
    ovr_GetInputState(session, ovrControllerType_Touch, &InputState);

    ovrVector2f rightStick = InputState.Thumbstick[ovrHand_Right];
    ovrVector2f leftStick = InputState.Thumbstick[ovrHand_Left];

    const float radialDeadZone = 0.5f;
    if (std::abs(leftStick.x) < radialDeadZone) leftStick.x = 0.0f;
    if (std::abs(leftStick.y) < radialDeadZone) leftStick.y = 0.0f;
    if (std::abs(rightStick.x) < radialDeadZone) rightStick.x = 0.0f;

    bool buttonPressed_A = (InputState.Buttons & ovrButton_A);
    bool wasButtonPressed_A = (LastInputState.Buttons & ovrButton_A);
    bool justPressedA = (!wasButtonPressed_A && buttonPressed_A);

    bool buttonPressed_B = (InputState.Buttons & ovrButton_B);
    bool wasButtonPressed_B = (LastInputState.Buttons & ovrButton_B);
    bool justPressedB = (!wasButtonPressed_B && buttonPressed_B);

    static float robot_stand = 0.0f;
    if (justPressedA && !robot_stand) robot_stand = 1.0f;
    if (justPressedB && robot_stand) robot_stand = 0.0f;

    float rightIndexTrigger = InputState.IndexTrigger[ovrHand_Right];
    float lastRightIndexTrigger = LastInputState.IndexTrigger[ovrHand_Right];
    float calibration = 0.0f;
    if (lastRightIndexTrigger < 0.5f && rightIndexTrigger >= 0.5f) {
        calibration = 1.0f;
    }

    float rightHandTrigger = InputState.HandTrigger[ovrHand_Right];
    float alignment = 0.0f;
    if (rightHandTrigger >= 0.5f) {
        alignment = 1.0f;
    }

    ovrTrackingState ts = ovr_GetTrackingState(session, ovr_GetTimeInSeconds(), ovrTrue);
    RPY orientation = quaternionToRPY(ts);

    // Fill data array
    data[0] = orientation.yaw;
    data[1] = -orientation.pitch;
    data[2] = -orientation.roll;
    data[3] = leftStick.y;
    data[4] = leftStick.x;
    data[5] = rightStick.x;
    data[6] = robot_stand;
    data[7] = calibration;
    data[8] = alignment;

    // Print on the same line, overwrite previous output, and pad with spaces to clear leftovers
    /*printf(
        "\rYaw:%6.2f | Pitch:%6.2f | Roll:%6.2f | LS(Y:%5.2f,X:%5.2f) | RS(X:%5.2f) | Stand:%.1f | Calib:%.1f | Algn:%.1f %-20s",
        orientation.yaw,
        -orientation.pitch,
        -orientation.roll,
        leftStick.y,
        leftStick.x,
        rightStick.x,
        robot_stand,
        calibration,
        alignment,
        "" // extra spaces to clear leftovers
    );*/

    // Update last input state
    LastInputState = InputState;
}


AppConfig LoadAppConfig(const std::string& filename) {
    std::ifstream config_file(filename);
    if (!config_file) {
        throw std::runtime_error("[ConfigLoader] Failed to open " + filename);
    }

    json config;
    config_file >> config;

    std::string ip_address = config.value("ip", "");
    std::string stream_address = config.value("stream", "");
    bool use_katvr = config.value("use_katvr", false);

    if (ip_address.empty()) {
        throw std::runtime_error("[ConfigLoader] Missing 'ip' in config.");
    }
    if (stream_address.empty()) {
        throw std::runtime_error("[ConfigLoader] Missing 'stream' in config.");
    }

    return { use_katvr, ip_address, stream_address };
}