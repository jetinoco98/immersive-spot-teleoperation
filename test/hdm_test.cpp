#include <cmath>
#include <thread>
#include <chrono>
#include <windows.h>

#include <Extras/OVR_Math.h>
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

#include <zmq.hpp>

#define M_PI 3.141592


struct RPY {
    double roll, pitch, yaw;
};

RPY quaternionToRPY_FullYaw(ovrTrackingState ts) {
    RPY rpy;
    double w = ts.HeadPose.ThePose.Orientation.w;
    double x = ts.HeadPose.ThePose.Orientation.x;
    double y = ts.HeadPose.ThePose.Orientation.y;
    double z = ts.HeadPose.ThePose.Orientation.z;

    // Pitch (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    rpy.pitch = std::atan2(sinr_cosp, cosr_cosp);

    // Yaw (y-axis rotation) — FULL RANGE [-180, 180]
    double siny_cosp = 2 * (w * y + z * x);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    rpy.yaw = std::atan2(siny_cosp, cosy_cosp);

    // Roll (z-axis rotation)
    double sinp = 2 * (w * z - x * y);
    if (std::abs(sinp) >= 1)
        rpy.roll = std::copysign(M_PI / 2, sinp);
    else
        rpy.roll = std::asin(sinp);

    return rpy;
}

int main() {

    // Execute python scripts
    printf("Executing oculus_client.py script...\n");
    char currentDir[MAX_PATH];
    GetCurrentDirectory(MAX_PATH, currentDir);
    printf(currentDir);

    const char* pythonScript = "\\..\\..\\scripts\\oculus\\oculus_test.py ";
    std::string command = "start cmd /k python \""
                        + std::string(currentDir) 
                        + std::string(pythonScript);

    // Initialize ZMQ Socket
    printf("Initializing ZMQ Socket...\n");
    zmq::context_t zmq_context(1);
    zmq::socket_t publisher(zmq_context, zmq::socket_type::pub);
    publisher.connect("tcp://localhost:5555");

    // Initialize libOVR (Oculus SDK)
    printf("Initializing Oculus SDK...\n");
    ovrResult result = ovr_Initialize(nullptr);
    if (OVR_FAILURE(result)) {
        printf("ERROR: Failed to initialize Oculus SDK");
        return -1;
    }

    ovrSession session;
    ovrGraphicsLuid luid;
    // Connect to the Oculus headset
    printf("Connecting to Oculus Rift...\n");
    result = ovr_Create(&session, &luid);
    if (OVR_FAILURE(result)) {
        printf("ERROR: Oculus Rift not detected");
        ovr_Shutdown();
        return -1;
    }

    ovrInputState InputState;
    ovrInputState LastInputState;
    float robot_stand = 0.0;

    RPY orientation;

    printf("Starting main loop...\n");
    while (true) {
        
        // ===================================================
        // Query the HMD for ts current tracking state.

        ovrTrackingState ts = ovr_GetTrackingState(session, ovr_GetTimeInSeconds(), ovrTrue);
        ovrResult inputResult = ovr_GetInputState(session, ovrControllerType_Active, &InputState);

        ovrVector2f rightStick = InputState.Thumbstick[ovrHand_Right];
        ovrVector2f leftStick = InputState.Thumbstick[ovrHand_Left];
        const float radialDeadZone = 0.5;
        if (std::abs(leftStick.x) < radialDeadZone) leftStick.x = 0.0;
        if (std::abs(leftStick.y) < radialDeadZone) leftStick.y = 0.0;
        if (std::abs(rightStick.x) < radialDeadZone) rightStick.x = 0.0;

        bool buttonPressed_A = ((InputState.Buttons & ovrButton_A) != 0);
        bool wasButtonPressed_A = ((LastInputState.Buttons & ovrButton_A) != 0);
        bool justPressedA = (!wasButtonPressed_A && buttonPressed_A);

        bool buttonPressed_B = ((InputState.Buttons & ovrButton_B) != 0);
        bool wasButtonPressed_B = ((LastInputState.Buttons & ovrButton_B) != 0);
        bool justPressedB = (!wasButtonPressed_B && buttonPressed_B);

        if (justPressedA && !robot_stand) robot_stand = 1.0;
        if (justPressedB && robot_stand) robot_stand = 0.0;
        
        orientation = quaternionToRPY_FullYaw(ts);

        float data[] = {orientation.yaw, -orientation.pitch, -orientation.roll, 
            leftStick.y, leftStick.x, rightStick.x, robot_stand};

        printf(
            "Yaw: %6.2f | Pitch: %6.2f | Roll: %6.2f | LS(Y: %5.2f, X: %5.2f) | RS(X: %5.2f) | A: %d | B: %d | robot_stand: %.1f\n",
            orientation.yaw,
            -orientation.pitch,
            -orientation.roll,
            leftStick.y,
            leftStick.x,
            rightStick.x,
            buttonPressed_A,
            buttonPressed_B,
            robot_stand 
        );
            
        LastInputState = InputState;

        // ===================================================
        // Send HDM data over ZeroMQ socket
		
        // Send the first part of the message (topic)
        std::string topic = "from_hdm"; 
        zmq::message_t topic_msg(topic.data(), topic.size());
        publisher.send(topic_msg, zmq::send_flags::sndmore);

        // Send the second part of the message (HDM data)
        zmq::message_t data_msg(sizeof(data)); 
        std::memcpy(data_msg.data(), data, sizeof(data));
        publisher.send(data_msg, zmq::send_flags::none);


        // ===================================================
        // Sleep for a while before querying again
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    return 0;
}