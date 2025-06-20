#include "main.hpp"

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
    // // Initialize the Oculus renderer
    if (!renderer.initialize(stream.getWidth(), stream.getHeight())) {
        std::cerr << "[Error] Unable to initialize Oculus renderer." << std::endl;
        return -1;
    }

    // Execute python scripts
    runPythonScript("\\..\\..\\scripts\\oculus\\oculus_client.py", ip_address);
    if (config.use_katvr) {
        runPythonScript("\\..\\..\\scripts\\katvr\\katvr.py");
    }

	// Initialize ZMQ Socket
    zmq::context_t zmq_context(1);
    zmq::socket_t publisher(zmq_context, zmq::socket_type::pub);
    publisher.connect("tcp://localhost:5555");

    // Create variables for HDM State 
    float data[10];

    printf("Starting main loop...\n");

    // ============================================================================
    //                                 MAIN LOOP
    // ============================================================================

    while (true) {

        // --- Update the Oculus renderer
        if (!renderer.update(stream.buffer_)) {break;}

        // --- Query the HMD for the input state (buttons, thumbsticks, etc.)
        getOculusInput(data, oculus_session);

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

    // --- CLOSING AND CLEANUP
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


// ================================================================
//                        FUNCTION DEFINITIONS
// ================================================================

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


void getOculusInput(float* data, ovrSession& session) {
    ovrInputState InputState;
    ovr_GetInputState(session, ovrControllerType_Touch, &InputState);

    ovrVector2f rightStick = InputState.Thumbstick[ovrHand_Right];
    ovrVector2f leftStick  = InputState.Thumbstick[ovrHand_Left];

    ovrTrackingState ts = ovr_GetTrackingState(session, ovr_GetTimeInSeconds(), ovrTrue);
    RPY orientation = quaternionToRPY(ts);

    // Pack orientation and thumbsticks
    data[0] = orientation.yaw;
    data[1] = -orientation.pitch;
    data[2] = -orientation.roll;
    data[3] = leftStick.x;
    data[4] = leftStick.y;
    data[5] = rightStick.x;
    data[6] = rightStick.y;

    // Buttons: A, B, X, Y
    data[7] = (InputState.Buttons & ovrButton_A) ? 1.0f : 0.0f;
    data[8] = (InputState.Buttons & ovrButton_B) ? 1.0f : 0.0f;
    data[9] = (InputState.Buttons & ovrButton_X) ? 1.0f : 0.0f;
    data[10] = (InputState.Buttons & ovrButton_Y) ? 1.0f : 0.0f;

    // Thumbstick clicks
    data[11] = (InputState.Buttons & ovrButton_LThumb) ? 1.0f : 0.0f;
    data[12] = (InputState.Buttons & ovrButton_RThumb) ? 1.0f : 0.0f;

    // Triggers
    data[13] = InputState.IndexTrigger[ovrHand_Left];
    data[14] = InputState.IndexTrigger[ovrHand_Right];
    data[15] = InputState.HandTrigger[ovrHand_Left];   // grip
    data[16] = InputState.HandTrigger[ovrHand_Right];  // grip
}


AppConfig LoadAppConfig(const std::string& filename) {
    std::ifstream config_file(filename);
    if (!config_file) {
        throw std::runtime_error("[ConfigLoader] Failed to open " + filename);
    }

    json config;
    config_file >> config;

    bool use_katvr = config.value("use_katvr", false);
    int ip_stream_value = config.value("ip_stream_value", 1);

    std::string ip_key = "ip_" + std::to_string(ip_stream_value);
    std::string stream_key = "stream_" + std::to_string(ip_stream_value);

    std::string ip_address = config.value(ip_key, "");
    std::string stream_address = config.value(stream_key, "");

    if (ip_address.empty()) {
        throw std::runtime_error("[ConfigLoader] Missing '" + ip_key + "' in config.");
    }
    if (stream_address.empty()) {
        throw std::runtime_error("[ConfigLoader] Missing '" + stream_key + "' in config.");
    }

    return { use_katvr, ip_address, stream_address };
}