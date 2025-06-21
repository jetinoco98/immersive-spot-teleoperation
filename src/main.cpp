#include "main.hpp"

#include <chrono>
#include <thread>  // To simulate a delay

// ============================================================================
//                                 MAIN FUNCTION
// ============================================================================

int main(int argc, char* argv[]) 
{
    // Loads the configuration file (config.json)
    AppConfig config = LoadAppConfig();

    // Initialize Oculus SDK and create a session
    ovrSession oculus_session = nullptr;
    if (!InitOculus(oculus_session)) return -1;

    // Initialize the stream capture process
    StreamCapture stream(config.stream_address);
    if (!stream.start()) return -1;

    // Initialize the Oculus renderer
    OculusRenderer renderer(oculus_session);
    if (!renderer.initialize(stream.width, stream.height)) return -1;
    
    // Execute python scripts
    runPythonScript("\\..\\..\\scripts\\oculus\\oculus_client.py", config.ip_address);
    if (config.use_katvr) {
        runPythonScript("\\..\\..\\scripts\\katvr\\katvr.py");
    }

	// Initialize ZMQ Socket
    ZMQPublisher zmq("tcp://localhost:5555");

    // Create data variable for HDM & Touch Controller State
    float data[17];

    printf("Starting main loop...\n");

    // ============================================================================
    //                                 MAIN LOOP
    // ============================================================================

    int loop_count = 0;
    auto last_time = std::chrono::steady_clock::now();

    while (true) {

        // --- Update the Oculus renderer
        if (!renderer.update(stream.buffer_)) {break;}

        // --- Query the HMD for the input state (buttons, thumbsticks, etc.)
        getOculusInput(data, oculus_session);

        // --- Send HDM data over ZeroMQ socket
        zmq.send("from_hdm", data, sizeof(data));

        // Print how many times the loop runs every second
        loop_count++;

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_time);

        if (elapsed.count() >= 1) {
            // std::cout << "Loop frequency: " << loop_count << " Hz" << std::endl;
            loop_count = 0;
            last_time = now;
        }

    }

    return 0;
}


// ================================================================
//                        FUNCTION DEFINITIONS
// ================================================================

bool InitOculus(ovrSession& session) {
    ovrResult result = ovr_Initialize(nullptr);
    if (OVR_FAILURE(result)) {
        std::cerr << "Failed to initialize Oculus SDK" << std::endl;
        return false;
    }

    ovrGraphicsLuid luid;
    result = ovr_Create(&session, &luid);
    if (OVR_FAILURE(result)) {
        std::cerr << "Failed to create Oculus session" << std::endl;
        ovr_Shutdown();
        return false;
    }
    return true;
}


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

    // Line 1: Orientation
    printf("\x1b[2K\rOrientation: (Yaw=%.2f Pitch=%.2f Roll=%.2f)         \n",
        data[0], data[1], data[2]);

    // Line 2: Buttons (A, B, X, Y, Left Thumb, Right Thumb)
    printf("\x1b[2K\rButtons: (X=%.0f Y=%.0f LT=%.0f)   (A=%.0f B=%.0f RT=%.0f)          \n",
        data[7], data[8], data[9], data[10], data[11], data[12]);

    // Line 3: Joysticks (Left and Right)
    printf("\x1b[2K\rJoysticks: (LStick X=%.2f Y=%.2f)   (RStick X=%.2f Y=%.2f)          \n",
        data[3], data[4], data[5], data[6]);

    // Line 4: Triggers and Grips (Left and Right)
    printf("\x1b[2K\rTriggers&Grips: (LTrigger=%.2f LGrip=%.2f)   (RTrigger=%.2f RGrip=%.2f)          \n",
        data[13], data[15], data[14], data[16]);

    // Move cursor up 4 lines for next overwrite
    printf("\x1b[4A");
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


// ================================================================
//                ZMQPublisher Class Implementation
// ================================================================

ZMQPublisher::ZMQPublisher(const std::string& address)
    : context_(1), publisher_(context_, zmq::socket_type::pub)
{
    publisher_.connect(address);
}

void ZMQPublisher::send(const std::string& topic, const void* data, size_t size) {
    zmq::message_t topic_msg(topic.data(), topic.size());
    zmq::message_t data_msg(size);
    std::memcpy(data_msg.data(), data, size);

    publisher_.send(topic_msg, zmq::send_flags::sndmore);
    publisher_.send(data_msg, zmq::send_flags::none);
}

ZMQPublisher::~ZMQPublisher() {
    publisher_.close();
    context_.shutdown();
    context_.close();
}