#pragma once

// OpenGL
#include <GL/glew.h>

// SDL
#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <SDL_syswm.h>

// Oculus SDK
#include <Extras/OVR_Math.h>
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

// OpenCV
#include <opencv2/opencv.hpp>

// Local includes
#include "Shader.hpp"
#include "StreamCapture.hpp"


class OculusRenderer {
public:
    OculusRenderer();
    ~OculusRenderer();

    bool initialize(int captureWidth, int captureHeight, ovrSession session);
    bool update(VideoCaptureFrameBuffer& buffer);
    void shutdown();

    ovrSession session_; // The current Oculus session

private:
    // Initialization Part 1
    void initializeSDL();
    void initializeGL();

    // Initialization Part 2
    void initializeCaptureTextures();
    void setEyeTextureSizes();
    void createTextureSwapChain();
    void initializeFrameBuffer();
    void initializeMirrorTexture();
    void initializeTracking();
    void initializeRectangleBuffers();
    void setupShaderAttributes();

    // Update 
    void grabFrame(VideoCaptureFrameBuffer& buffer);
    void renderToOculus(VideoCaptureFrameBuffer& buffer);

    // Internal functions
    static void BindCVMat2GLTexture(cv::Mat& image, GLuint& imageTexture);

    // Internal variables
    bool initialized_;  // Represents the initialization of the whole class
    SDL_Window* window_;
    SDL_GLContext glContext_;
    SDL_Event events_;   // SDL variable that will be used to store input events
    GLuint captureTextureID_[2];
    GLuint fboID_, depthBuffID_, mirrorFBOID_;
    ovrSizei textureSize0_, textureSize1_;
    ovrSizei bufferSize_;
    ovrPosef eyeRenderPose_[2];
    ovrPosef hmdToEyeOffset_[2];
    ovrHmdDesc hmdDesc_;
    GLuint rectVBO_[3];
    ovrTextureSwapChain textureChain_;
    std::unique_ptr<Shader> shader_;
    ovrMirrorTexture mirrorTexture_ = nullptr;

    // For Oculus SDK result and error checking
    ovrResult result;
    ovrErrorInfo errInf;

    int winWidth_, winHeight_;
    int captureWidth_, captureHeight_;
    long long frameIndex_;
    double sensorSampleTime_;
    bool isVisible_;
};