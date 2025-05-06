#define NOMINMAX

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <Windows.h>
#include <cmath>
#include <stddef.h>

#include <GL/glew.h>

#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <SDL_syswm.h>

#include <Extras/OVR_Math.h>
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

#include <sl/Camera.hpp>

#include <opencv2/opencv.hpp>

#include <zmq.hpp>

#include "Shader.hpp"
#include "json.hpp"


using json = nlohmann::json;
using namespace std;

bool connected = true;

// ===================================================
// ----- SHADER STRINGS -----
// ===================================================

GLchar* OVR_ZED_VS =
"#version 330 core\n \
			layout(location=0) in vec3 in_vertex;\n \
			layout(location=1) in vec2 in_texCoord;\n \
			uniform uint isLeft; \n \
			out vec2 b_coordTexture; \n \
			void main()\n \
			{\n \
				if (isLeft == 1U)\n \
				{\n \
					b_coordTexture = in_texCoord;\n \
					gl_Position = vec4(in_vertex.x, in_vertex.y, in_vertex.z,1);\n \
				}\n \
				else \n \
				{\n \
					b_coordTexture = vec2(1.0 - in_texCoord.x, in_texCoord.y);\n \
					gl_Position = vec4(-in_vertex.x, in_vertex.y, in_vertex.z,1);\n \
				}\n \
			}";

GLchar* OVR_ZED_FS =
"#version 330 core\n \
			uniform sampler2D u_textureZED; \n \
			in vec2 b_coordTexture;\n \
			out vec4 out_color; \n \
			void main()\n \
			{\n \
				out_color = vec4(texture(u_textureZED, b_coordTexture).bgr,1); \n \
			}";


// ===================================================
// ----- OpenCV to OpenGL FUNCTION -----
// ===================================================
// This function binds an OpenCV matrix (cv::Mat) to an OpenGL texture,
// allowing it to be used in 3D rendering and other OpenGL operations.

void BindCVMat2GLTexture(cv::Mat& image, GLuint& imageTexture)
{
    if (image.empty()) {
        std::cout << "Image is empty" << std::endl;
        return;
    }

    // Convert from BGR to RGB once if needed
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    glGenTextures(1, &imageTexture);
    glBindTexture(GL_TEXTURE_2D, imageTexture);

    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGB,
                 image.cols,
                 image.rows,
                 0,
                 GL_RGB,
                 GL_UNSIGNED_BYTE,
                 image.ptr());

    glBindTexture(GL_TEXTURE_2D, 0); // Optional: unbind for safety
}

// ===================================================
// ----- VIDEO CAPTURE THREAD -----
// ===================================================

// Packed data for threaded computation
struct ThreadData {
    std::mutex mtx;
    cv::Mat leftImage;
    cv::Mat rightImage;
    bool run;
    bool new_frame;
};

// Video capture thread
void __capture_runner__(ThreadData& thread_data, cv::VideoCapture cv_capture) {
    cv::Mat frame;
    auto last_success = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(5); // Customize the timeout

    // Loop while the main loop is not over
    while (thread_data.run) {
        // try to grab a new image
        if (cv_capture.read(frame)) {
            last_success = std::chrono::steady_clock::now(); // Reset the timer

            // copy both left and right images
            thread_data.mtx.lock();
            // Define the first ROI (left part of the frame)
            cv::Rect roi1(0, 0, frame.cols / 2, frame.rows);
            thread_data.leftImage = frame(roi1).clone(); // Use clone() to copy the data

            // Define the second ROI (right part of the frame)
            cv::Rect roi2(frame.cols / 2, 0, frame.cols / 2, frame.rows);
            thread_data.rightImage = frame(roi2).clone(); // Use clone() to copy the data

            thread_data.mtx.unlock();
            thread_data.new_frame = true;
        }
        else
            sl::sleep_ms(2);
        // Check for timeout
        auto now = std::chrono::steady_clock::now();
        if (now - last_success > timeout) {
            std::cerr << "[ERROR] No frame received for 5 seconds. Stopping capture thread." << std::endl;
            connected = false;
            Sleep(5000);
            std::exit(EXIT_FAILURE);
        }
    }
}


// ===================================================
// ----- RPY CONVERTER -----
// ===================================================

struct RPY {
    double roll, pitch, yaw;
};

RPY quaternionToRPY(ovrTrackingState ts) {
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


// ===================================================
// ----- MAIN FUNCTION -----
// ===================================================

int main(int argc, char* argv[])  {

    // Obtain variables from JSON file
    std::ifstream config_file("config.json");
    if (!config_file) {
        std::cerr << "Failed to open config.json\n";
        return 1;
    }

    json config;
    config_file >> config;

    std::string ip_address = config.value("ip", "48.209.18.239");
    std::string stream_address = config.value("stream_address", "48.209.18.239:8554/spot-stream");
    bool use_katvr = config.value("use_katvr", false); // Default to false if not specified


    // Initialize SDL2's context
    SDL_Init(SDL_INIT_VIDEO);
    // Initialize Oculus' context
    ovrResult result = ovr_Initialize(nullptr);
    if (OVR_FAILURE(result)) {
        std::cout << "ERROR: Failed to initialize libOVR" << std::endl;
        SDL_Quit();
        return -1;
    }

    ovrSession session;
    ovrGraphicsLuid luid;
    // Connect to the Oculus headset
    result = ovr_Create(&session, &luid);
    if (OVR_FAILURE(result)) {
        std::cout << "ERROR: Oculus Rift not detected" << std::endl;
        ovr_Shutdown();
        SDL_Quit();
        return -1;
    }

    int x = SDL_WINDOWPOS_CENTERED, y = SDL_WINDOWPOS_CENTERED;
    int winWidth = 1280;
    int winHeight = 720;
    Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;
    // Create SDL2 Window
    SDL_Window* window = SDL_CreateWindow("Stereo Passthrough", x, y, winWidth, winHeight, flags);
    // Create OpenGL context
    SDL_GLContext glContext = SDL_GL_CreateContext(window);
    // Initialize GLEW
    glewInit();
    // Turn off vsync to let the compositor do its magic
    SDL_GL_SetSwapInterval(0);

    // Create a struct which contains the sl::Camera and the associated data
    ThreadData thread_data;

    // Create the OpenCV video capture
    // std::string pipeline = "rtspsrc location=rtsp://" + stream_address + " latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! videoscale ! video/x-raw,width=1280,height=360,format=BGR ! appsink";
    // std::string pipeline = "rtspsrc location=rtsp://" + stream_address + " latency=100 ! rtph264depay ! h264parse ! nvh264dec ! videoconvert ! appsink";
    std::string pipeline = "rtspsrc location=rtsp://" + stream_address + " latency=100 ! rtph264depay ! h264parse ! decodebin ! videoconvert ! video/x-raw, format=BGR ! appsink";

    cv::VideoCapture cv_capture(pipeline, cv::CAP_GSTREAMER);

    // Check if the video stream is opened successfully
    if (!cv_capture.isOpened()) {
        std::cout << "Failed to open video stream or file!" << std::endl;
        cv_capture.release();
        ovr_Destroy(session);
        ovr_Shutdown();
        return -1; // Exit if the stream or file is not opened
    }

    auto captureWidth = static_cast<int>(cv_capture.get(cv::CAP_PROP_FRAME_WIDTH));
    auto captureHeight = static_cast<int>(cv_capture.get(cv::CAP_PROP_FRAME_HEIGHT));

    //sl::uchar4 dark_bckgrd(44, 44, 44, 255);
    GLuint captureTextureID[2];
    glGenTextures(2, captureTextureID);
    for (int eye = 0; eye < 2; eye++) {
        // Generate OpenGL texture
        glBindTexture(GL_TEXTURE_2D, captureTextureID[eye]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, captureWidth, captureHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }


    float pixel_density = 1.75f;
    ovrHmdDesc hmdDesc = ovr_GetHmdDesc(session);
    ovrInputState InputState;
    ovrInputState LastInputState;
    float robot_stand = 0.0;
    float calibrate_vr = 0.0;

    // Get the texture sizes of Oculus eyes
    ovrSizei textureSize0 = ovr_GetFovTextureSize(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0], pixel_density);
    ovrSizei textureSize1 = ovr_GetFovTextureSize(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1], pixel_density);
    // Compute the final size of the render buffer
    ovrSizei bufferSize;
    bufferSize.w = textureSize0.w + textureSize1.w;
    bufferSize.h = std::max(textureSize0.h, textureSize1.h);

    // Initialize OpenGL swap textures to render
    ovrTextureSwapChain textureChain = nullptr;
    // Description of the swap chain
    ovrTextureSwapChainDesc descTextureSwap = {};
    descTextureSwap.Type = ovrTexture_2D;
    descTextureSwap.ArraySize = 1;
    descTextureSwap.Width = bufferSize.w;
    descTextureSwap.Height = bufferSize.h;
    descTextureSwap.MipLevels = 1;
    descTextureSwap.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
    descTextureSwap.SampleCount = 1;
    descTextureSwap.StaticImage = ovrFalse;
    // Create the OpenGL texture swap chain
    result = ovr_CreateTextureSwapChainGL(session, &descTextureSwap, &textureChain);

    ovrErrorInfo errInf;
    if (OVR_SUCCESS(result)) {
        int length = 0;
        ovr_GetTextureSwapChainLength(session, textureChain, &length);
        for (int i = 0; i < length; ++i) {
            GLuint chainTexId;
            ovr_GetTextureSwapChainBufferGL(session, textureChain, i, &chainTexId);
            glBindTexture(GL_TEXTURE_2D, chainTexId);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        }
    }
    else {
        ovr_GetLastErrorInfo(&errInf);
        std::cout << "ERROR: failed creating swap texture " << errInf.ErrorString << std::endl;
        ovr_Destroy(session);
        ovr_Shutdown();
        SDL_GL_DeleteContext(glContext);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }
    // Generate frame buffer to render
    GLuint fboID;
    glGenFramebuffers(1, &fboID);
    // Generate depth buffer of the frame buffer
    GLuint depthBuffID;
    glGenTextures(1, &depthBuffID);
    glBindTexture(GL_TEXTURE_2D, depthBuffID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    GLenum internalFormat = GL_DEPTH_COMPONENT24;
    GLenum type = GL_UNSIGNED_INT;
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, bufferSize.w, bufferSize.h, 0, GL_DEPTH_COMPONENT, type, NULL);

    // Create a mirror texture to display the render result in the SDL2 window
    ovrMirrorTextureDesc descMirrorTexture;
    memset(&descMirrorTexture, 0, sizeof(descMirrorTexture));
    descMirrorTexture.Width = winWidth;
    descMirrorTexture.Height = winHeight;
    descMirrorTexture.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;

    ovrMirrorTexture mirrorTexture = nullptr;
    result = ovr_CreateMirrorTextureGL(session, &descMirrorTexture, &mirrorTexture);
    if (!OVR_SUCCESS(result)) {
        ovr_GetLastErrorInfo(&errInf);
        std::cout << "ERROR: Failed to create mirror texture " << errInf.ErrorString << std::endl;
    }
    GLuint mirrorTextureId;
    ovr_GetMirrorTextureBufferGL(session, mirrorTexture, &mirrorTextureId);

    GLuint mirrorFBOID;
    glGenFramebuffers(1, &mirrorFBOID);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBOID);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mirrorTextureId, 0);
    glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    // Frame index used by the compositor, it needs to be updated each new frame
    long long frameIndex = 0;

    // FloorLevel will give tracking poses where the floor height is 0
    ovr_SetTrackingOriginType(session, ovrTrackingOrigin_FloorLevel);

    // Initialize a default Pose
    ovrPosef eyeRenderPose[2];
    ovrPosef hmdToEyeOffset[2];

    // Get the Oculus view scale description
    double sensorSampleTime;

    // Create and compile the shader's sources
    Shader shader(OVR_ZED_VS, OVR_ZED_FS);

    // Compute the useful part of the ZED image
    unsigned int widthFinal = bufferSize.w / 2;
    float heightGL = 1.f;
    float widthGL = 1.f;
    if (captureWidth > 0.f) {
        unsigned int heightFinal = captureHeight * widthFinal / (float)captureWidth;
        // Convert this size to OpenGL viewport's frame's coordinates
        heightGL = (heightFinal) / (float)(bufferSize.h);
        widthGL = ((captureWidth * (heightFinal / (float)captureHeight)) / (float)widthFinal);
    }
    else {
        std::cout << "WARNING: Video capture parameters got wrong values."
            "Default vertical and horizontal FOV are used."
            << std::endl;
    }

    // Compute the Horizontal Oculus' field of view with its parameters
    float ovrFovH = (atanf(hmdDesc.DefaultEyeFov[0].LeftTan) + atanf(hmdDesc.DefaultEyeFov[0].RightTan));
    // Compute the Vertical Oculus' field of view with its parameters
    float ovrFovV = (atanf(hmdDesc.DefaultEyeFov[0].UpTan) + atanf(hmdDesc.DefaultEyeFov[0].DownTan));

    // Compute the center of the optical lenses of the headset
    float offsetLensCenterX = ((atanf(hmdDesc.DefaultEyeFov[0].LeftTan)) / ovrFovH) * 2.f - 1.f;
    float offsetLensCenterY = ((atanf(hmdDesc.DefaultEyeFov[0].UpTan)) / ovrFovV) * 2.f - 1.f;

    // Create a rectangle with the computed coordinates and push it in GPU memory
    struct GLScreenCoordinates {
        float left, up, right, down;
    } screenCoord;

    screenCoord.up = heightGL + offsetLensCenterY;
    screenCoord.down = heightGL - offsetLensCenterY;
    screenCoord.right = widthGL * .75 + offsetLensCenterX;
    screenCoord.left = widthGL * .75 - offsetLensCenterX;
    

    float rectVertices[12] = { -screenCoord.left, -screenCoord.up, 0, screenCoord.right, -screenCoord.up, 0, screenCoord.right, screenCoord.down, 0, -screenCoord.left, screenCoord.down, 0 };
    GLuint rectVBO[3];
    glGenBuffers(1, &rectVBO[0]);
    glBindBuffer(GL_ARRAY_BUFFER, rectVBO[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(rectVertices), rectVertices, GL_STATIC_DRAW);

    float rectTexCoord[8] = { 0, 1, 1, 1, 1, 0, 0, 0 };
    glGenBuffers(1, &rectVBO[1]);
    glBindBuffer(GL_ARRAY_BUFFER, rectVBO[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(rectTexCoord), rectTexCoord, GL_STATIC_DRAW);

    unsigned int rectIndices[6] = { 0, 1, 2, 0, 2, 3 };
    glGenBuffers(1, &rectVBO[2]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rectVBO[2]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(rectIndices), rectIndices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Initialize a boolean that will be used to stop the application�s loop and another one to pause/unpause rendering
    bool end = false;
    // SDL variable that will be used to store input events
    SDL_Event events;
    // This boolean is used to test if the application is focused
    bool isVisible = true;

    // Enable the shader
    glUseProgram(shader.getProgramId());
    // Bind the Vertex Buffer Objects of the rectangle that displays video capture images

    // vertices
    glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);
    glBindBuffer(GL_ARRAY_BUFFER, rectVBO[0]);
    glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
    // indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rectVBO[2]);
    // texture coordinates
    glEnableVertexAttribArray(Shader::ATTRIB_TEXTURE2D_POS);
    glBindBuffer(GL_ARRAY_BUFFER, rectVBO[1]);
    glVertexAttribPointer(Shader::ATTRIB_TEXTURE2D_POS, 2, GL_FLOAT, GL_FALSE, 0, 0);

    // Set thread variables
    thread_data.run = true;
    thread_data.new_frame = true;
    // Launch capture video thread
    std::thread runner(__capture_runner__, std::ref(thread_data), cv_capture);

    // Execute python scripts
    printf("Executing oculus_client.py script...\n");
    char currentDir[MAX_PATH];
    GetCurrentDirectory(MAX_PATH, currentDir);
    printf(currentDir);

    const char* pythonScript = "\\..\\..\\scripts\\oculus\\oculus_client.py ";
    std::string command = "start cmd /k python \""
                        + std::string(currentDir) 
                        + std::string(pythonScript) 
                        + "\" " + ip_address;

    if (system(command.c_str()) != 0) {
        std::cerr << "Error executing Oculus Python script" << std::endl;
        return 1;
    }

    // Execute additional python scripts
    if (use_katvr) {
        printf("Executing katvr_main.py script...\n");
        
        const char* katvrScript = "\\..\\..\\scripts\\katvr\\katvr_main.py ";
        std::string katvrCommand = "start cmd /k python \""
                                + std::string(currentDir) 
                                + std::string(katvrScript) 
                                + "\"";

        if (system(katvrCommand.c_str()) != 0) {
            std::cerr << "Error executing KATVR Python script" << std::endl;
            return 1;
        }
    }

	// Initialize ZMQ Socket
    zmq::context_t zmq_context(1);
    zmq::socket_t publisher(zmq_context, zmq::socket_type::pub);
    publisher.connect("tcp://localhost:5555");

    // Create orientation structure
    RPY orientation;

    // ===================================================
    // ----- MAIN LOOP -----
    // ===================================================

    while (!end) {
        // While there is an event catched and not tested
        while (SDL_PollEvent(&events)) {
            // If a key is released
            if (events.type == SDL_KEYUP) {
                // If Q -> quit the application
                if (events.key.keysym.scancode == SDL_SCANCODE_Q)
                    end = true;
            }
        }

        // Get texture swap index where we must draw our frame
        GLuint curTexId;
        int curIndex;
        ovr_GetTextureSwapChainCurrentIndex(session, textureChain, &curIndex);
        ovr_GetTextureSwapChainBufferGL(session, textureChain, curIndex, &curTexId);

        // Call ovr_GetRenderDesc each frame to get the ovrEyeRenderDesc, as the returned values (e.g. HmdToEyeOffset) may change at runtime.
        hmdToEyeOffset[ovrEye_Left] = ovr_GetRenderDesc(session, ovrEye_Left, hmdDesc.DefaultEyeFov[ovrEye_Left]).HmdToEyePose;
        hmdToEyeOffset[ovrEye_Right] = ovr_GetRenderDesc(session, ovrEye_Right, hmdDesc.DefaultEyeFov[ovrEye_Right]).HmdToEyePose;

        // Get eye poses, feeding in correct IPD offset
        ovr_GetEyePoses2(session, frameIndex, ovrTrue, hmdToEyeOffset, eyeRenderPose, &sensorSampleTime);

        // If the application is focused
        if (isVisible) {
            // If successful grab a new capture image
            if (thread_data.new_frame) {
                // Set the flag to false
                thread_data.new_frame = false;

                BindCVMat2GLTexture(thread_data.leftImage, captureTextureID[0]);
                BindCVMat2GLTexture(thread_data.rightImage, captureTextureID[1]);

                // Bind the frame buffer
                glBindFramebuffer(GL_FRAMEBUFFER, fboID);
                // Set its color layer 0 as the current swap texture
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, curTexId, 0);
                // Set its depth layer as our depth buffer
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthBuffID, 0);
                // Clear the frame buffer
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                glClearColor(0, 0, 0, 1);

                // Render for each Oculus eye the equivalent video capture image
                for (int eye = 0; eye < 2; eye++) {
                    // Set the left or right vertical half of the buffer as the viewport
                    glViewport(eye == ovrEye_Left ? 0 : bufferSize.w / 2, 0, bufferSize.w / 2, bufferSize.h);
                    // Bind the left or right video capture image
                    glBindTexture(GL_TEXTURE_2D, eye == ovrEye_Left ? captureTextureID[ovrEye_Left] : captureTextureID[ovrEye_Right]);
                    // Bind the isLeft value
                    glUniform1ui(glGetUniformLocation(shader.getProgramId(), "isLeft"), eye == ovrEye_Left ? 1U : 0U);
                    // Draw the video capture image
                    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
                }
                // Avoids an error when calling SetAndClearRenderSurface during next iteration.
                // Without this, during the next while loop iteration SetAndClearRenderSurface
                // would bind a framebuffer with an invalid COLOR_ATTACHMENT0 because the texture ID
                // associated with COLOR_ATTACHMENT0 had been unlocked by calling wglDXUnlockObjectsNV.
                glBindFramebuffer(GL_FRAMEBUFFER, fboID);
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
                // Commit changes to the textures so they get picked up frame
                ovr_CommitTextureSwapChain(session, textureChain);
            }
            // Do not forget to increment the frameIndex!
            frameIndex++;
        }

        /*
        Note: Even if we don't ask to refresh the framebuffer or if the Camera::grab()
              doesn't catch a new frame, we have to submit an image to the Rift; it
                  needs 75Hz refresh. Else there will be jumbs, black frames and/or glitches
                  in the headset.
         */
        ovrLayerEyeFov ld;
        ld.Header.Type = ovrLayerType_EyeFov;
        // Tell to the Oculus compositor that our texture origin is at the bottom left
        ld.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft; // Because OpenGL | Disable head tracking
        // Set the Oculus layer eye field of view for each view
        for (int eye = 0; eye < 2; ++eye) {
            // Set the color texture as the current swap texture
            ld.ColorTexture[eye] = textureChain;
            // Set the viewport as the right or left vertical half part of the color texture
            ld.Viewport[eye] = OVR::Recti(eye == ovrEye_Left ? 0 : (bufferSize.w / 2) + 200, 0, (bufferSize.w / 2) - 200, bufferSize.h);
            // Set the field of view
            ld.Fov[eye] = hmdDesc.DefaultEyeFov[eye];
            // Set the pose matrix
            ld.RenderPose[eye] = eyeRenderPose[eye];
        }

        ld.SensorSampleTime = sensorSampleTime;

        ovrLayerHeader* layers = &ld.Header;
        // Submit the frame to the Oculus compositor
        // which will display the frame in the Oculus headset
        result = ovr_SubmitFrame(session, frameIndex, nullptr, &layers, 1);

        if (!OVR_SUCCESS(result)) {
            ovr_GetLastErrorInfo(&errInf);
            std::cout << "ERROR: failed to submit frame " << errInf.ErrorString << std::endl;
            end = true;
        }

        if (result == ovrSuccess && !isVisible) {
            std::cout << "The application is now shown in the headset." << std::endl;
        }
        isVisible = (result == ovrSuccess);

        // This is not really needed for this application but it may be useful for an more advanced application
        ovrSessionStatus sessionStatus;
        ovr_GetSessionStatus(session, &sessionStatus);
        if (sessionStatus.ShouldRecenter) {
            std::cout << "Recenter Tracking asked by Session" << std::endl;
            ovr_RecenterTrackingOrigin(session);
        }

        // Copy the frame to the mirror buffer which will be drawn in the SDL2 image
        glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBOID);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        GLint w = winWidth;
        GLint h = winHeight;
        glBlitFramebuffer(0, h, w, 0,
            0, 0, w, h,
            GL_COLOR_BUFFER_BIT, GL_NEAREST);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        // Swap the SDL2 window
        SDL_GL_SwapWindow(window);


        // ===================================================
        // Query the HMD for its current tracking state (ts)

		ovrTrackingState ts = ovr_GetTrackingState(session, ovr_GetTimeInSeconds(), ovrTrue);
        ovr_GetInputState(session, ovrControllerType_Touch, &InputState);
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

        float rightTrigger = InputState.IndexTrigger[ovrHand_Right];
        float lastRightTrigger = LastInputState.IndexTrigger[ovrHand_Right];
        bool justPressedTrigger = (lastRightTrigger < 0.5 && rightTrigger >= 0.5);

        if (justPressedA && !robot_stand) robot_stand = 1.0;
        if (justPressedB && robot_stand) robot_stand = 0.0;

        if (justPressedTrigger) {calibrate_vr = 1.0f;} 
        else {calibrate_vr = 0.0f;}

        orientation = quaternionToRPY(ts);

        float data[] = {orientation.yaw, -orientation.pitch, -orientation.roll, 
            leftStick.y, leftStick.x, rightStick.x, robot_stand, calibrate_vr};

        printf(
            "Yaw: %6.2f | Pitch: %6.2f | Roll: %6.2f | LS(Y: %5.2f, X: %5.2f) | RS(X: %5.2f) | STAND: %.1f | CALIBRATE: %.1f\n",
            orientation.yaw,
            -orientation.pitch,
            -orientation.roll,
            leftStick.y,
            leftStick.x,
            rightStick.x,
            robot_stand,
            calibrate_vr
        );

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

        // Update the last input state
        LastInputState = InputState;
    }

    
    // ===================================================
    // ----- CLOSING AND CLEANUP -----
    // ===================================================

    // Closing ZeroMQ socket
    publisher.close();
    zmq_context.shutdown();
    zmq_context.close();

    // Close the cv video capture context
    cv_capture.release();
    cv::destroyAllWindows();

    // Disable all OpenGL buffer
    glDisableVertexAttribArray(Shader::ATTRIB_TEXTURE2D_POS);
    glDisableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glUseProgram(0);
    glBindVertexArray(0);
    // Delete the Vertex Buffer Objects of the rectangle
    glDeleteBuffers(3, rectVBO);
    // Delete SDL, OpenGL and Oculus context
    ovr_DestroyTextureSwapChain(session, textureChain);
    ovr_DestroyMirrorTexture(session, mirrorTexture);
    ovr_Destroy(session);
    ovr_Shutdown();
    SDL_GL_DeleteContext(glContext);
    SDL_DestroyWindow(window);
    SDL_Quit();

    // Quit
    return 0;
}