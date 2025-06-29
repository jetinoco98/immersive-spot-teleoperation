#include "OculusRenderer.hpp"
#include <iostream>

// ============================================================================
//                           Constructor / Destructor
// ============================================================================

OculusRenderer::OculusRenderer(ovrSession& session)
    : session_(session)  // initialize reference member
{
}

OculusRenderer::~OculusRenderer() 
{
    shutdown();
}

// ============================================================================
//                                 Public Methods
// ============================================================================

bool OculusRenderer::initialize(int captureWidth, int captureHeight) 
{
    if (initialized_) return true;

    try
    {
        // Save the variables passed to the function
        captureWidth_ = captureWidth;
        captureHeight_ = captureHeight;

        initializeSDL();
        printf("SDL initialized successfully.\n");

        initializeGL();
        printf("OpenGL initialized successfully.\n");

        initializeCaptureTextures();
        printf("Capture textures initialized successfully.\n");

        setEyeTextureSizes();
        printf("Eye texture sizes set successfully.\n");

        createTextureSwapChain();
        printf("Texture swap chain created successfully.\n");

        initializeFrameBuffer();
        printf("Frame buffer initialized successfully.\n");

        initializeMirrorTexture();
        printf("Mirror texture initialized successfully.\n");

        initializeTracking();
        printf("Tracking initialized successfully.\n");

        initializeRectangleBuffers();
        printf("Rectangle buffers initialized successfully.\n");

        printf("Creating shader...\n");
        shader_ = std::make_unique<Shader>(OVR_ZED_VS, OVR_ZED_FS);

        setupShaderAttributes();
        printf("Shader attributes set up successfully.\n");
    }
    catch(const std::exception& e)
    {
        printf("Failed to initialize Oculus renderer: %s\n", e.what());
        return false;
    }
    
    initialized_ = true;
    return true;
}


bool OculusRenderer::update(VideoCaptureFrameBuffer& buffer) 
{
    if (!initialized_) return false;

    while (SDL_PollEvent(&events_)) {
        // If a key is released
        if (events_.type == SDL_KEYUP) {
            // If Q -> quit the application
            if (events_.key.keysym.scancode == SDL_SCANCODE_Q)
                return false;
        }
    }

    try
    {
        grabFrame(buffer);
        renderToOculus(buffer);
    }
    catch(const std::exception& e)
    {
        printf("Failed to update Oculus renderer: %s\n", e.what());
        return false;
    }
    
    return true;
}


// ============================================================================
//                   Private Methods - Helper Function
// ============================================================================

void OculusRenderer::BindCVMat2GLTexture(cv::Mat& image, GLuint& imageTexture)
{
    if (image.empty()) {
        std::cout << "Image is empty" << std::endl;
        return;
    }

    // Convert image to RGB format
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
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


void OculusRenderer::shutdown() 
{
    if (!initialized_) return;

    // Cleanup Oculus SDK OVR 
    try
    {
        ovr_DestroyTextureSwapChain(session_, textureChain_);
        ovr_DestroyMirrorTexture(session_, mirrorTexture_);
        ovr_Destroy(session_);
        ovr_Shutdown();
    }
    catch(const std::exception& e)
    {
        printf("Failed to destroy Oculus session: %s\n", e.what());
    }

    // Cleanup SDL
    try
    {
        SDL_GL_DeleteContext(glContext_);
        SDL_DestroyWindow(window_);
        SDL_Quit();
    }
    catch(const std::exception& e)
    {
        printf("Failed to cleanup SDL context: %s\n", e.what());
    }

    // Cleanup OpenGL
    try
    {
        // Disable all OpenGL buffer
        glDisableVertexAttribArray(Shader::ATTRIB_TEXTURE2D_POS);
        glDisableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glUseProgram(0);
        glBindVertexArray(0);
        // Delete the Vertex Buffer Objects of the rectangle
        glDeleteBuffers(3, rectVBO_);
    }
    catch(const std::exception& e)
    {
        printf("Failed to cleanup OpenGL context: %s\n", e.what());
    }
}


// ============================================================================
//                     Private Methods - Initialization
// ============================================================================

void OculusRenderer::initializeSDL() 
{
    // Initialize SDL2
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        throw std::runtime_error("Failed to initialize SDL");
    }

    // Create SDL2 Window
    int x = SDL_WINDOWPOS_CENTERED, y = SDL_WINDOWPOS_CENTERED;
    winWidth_ = 1280;
    winHeight_ = 720;
    Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;
    window_ = SDL_CreateWindow("Stereo Passthrough", x, y, winWidth_, winHeight_, flags);

    if (!window_) {
        throw std::runtime_error("Failed to create SDL window");
    }
    
    // Create OpenGL context
    glContext_ = SDL_GL_CreateContext(window_);

    if (!glContext_) {
        throw std::runtime_error("Failed to create OpenGL context");
    }
}


void OculusRenderer::initializeGL() 
{
    // Initialize GLEW
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        throw std::runtime_error("Failed to initialize GLEW (OpenGL Extension)");
    }
    // Turn off vsync to let the compositor do its magic
    SDL_GL_SetSwapInterval(0);

}


void OculusRenderer::initializeCaptureTextures() 
{
    glGenTextures(2, captureTextureID_);

    for (int eye = 0; eye < 2; eye++) {
        // Generate OpenGL texture
        glBindTexture(GL_TEXTURE_2D, captureTextureID_[eye]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, captureWidth_, captureHeight_, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
}
    

void OculusRenderer::setEyeTextureSizes() 
{
    float pixel_density = 1.75f;
    hmdDesc_ = ovr_GetHmdDesc(session_);
    textureSize0_ = ovr_GetFovTextureSize(session_, ovrEye_Left, hmdDesc_.DefaultEyeFov[0], pixel_density);
    textureSize1_ = ovr_GetFovTextureSize(session_, ovrEye_Right, hmdDesc_.DefaultEyeFov[1], pixel_density);
}

void OculusRenderer::createTextureSwapChain() 
{
    // Compute the final size of the render buffer
    bufferSize_.w = textureSize0_.w + textureSize1_.w;
    bufferSize_.h = std::max(textureSize0_.h, textureSize1_.h);

    // Initialize OpenGL swap textures to render
    textureChain_ = nullptr;
    // Description of the swap chain
    ovrTextureSwapChainDesc descTextureSwap = {};
    descTextureSwap.Type = ovrTexture_2D;
    descTextureSwap.ArraySize = 1;
    descTextureSwap.Width = bufferSize_.w;
    descTextureSwap.Height = bufferSize_.h;
    descTextureSwap.MipLevels = 1;
    descTextureSwap.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
    descTextureSwap.SampleCount = 1;
    descTextureSwap.StaticImage = ovrFalse;
    // Create the OpenGL texture swap chain
    ovrErrorInfo errInf;
    ovrResult result = ovr_CreateTextureSwapChainGL(session_, &descTextureSwap, &textureChain_);

    if (OVR_SUCCESS(result)) {
        int length = 0;
        ovr_GetTextureSwapChainLength(session_, textureChain_, &length);
        for (int i = 0; i < length; ++i) {
            GLuint chainTexId;
            ovr_GetTextureSwapChainBufferGL(session_, textureChain_, i, &chainTexId);
            glBindTexture(GL_TEXTURE_2D, chainTexId);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        }
    }
    else {
        ovr_GetLastErrorInfo(&errInf);
        throw std::runtime_error(std::string("Failed creating swap texture ") + errInf.ErrorString);
    }
}


void OculusRenderer::initializeFrameBuffer() 
{
    // Generate frame buffer to render
    glGenFramebuffers(1, &fboID_);
    // Generate depth buffer of the frame buffer
    glGenTextures(1, &depthBuffID_);
    glBindTexture(GL_TEXTURE_2D, depthBuffID_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    GLenum internalFormat = GL_DEPTH_COMPONENT24;
    GLenum type = GL_UNSIGNED_INT;
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, bufferSize_.w, bufferSize_.h, 0, GL_DEPTH_COMPONENT, type, NULL);
}


void OculusRenderer::initializeMirrorTexture() 
{
    // Create a mirror texture to display the render result in the SDL2 window
    ovrMirrorTextureDesc descMirrorTexture;
    memset(&descMirrorTexture, 0, sizeof(descMirrorTexture));
    descMirrorTexture.Width = winWidth_;
    descMirrorTexture.Height = winHeight_;
    descMirrorTexture.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;

    ovrErrorInfo errInf;
    ovrResult result = ovr_CreateMirrorTextureGL(session_, &descMirrorTexture, &mirrorTexture_);

    if (!OVR_SUCCESS(result)) {
        ovr_GetLastErrorInfo(&errInf);
        throw std::runtime_error(std::string("Failed to create mirror texture: ") + errInf.ErrorString);
    }
    GLuint mirrorTextureId;
    ovr_GetMirrorTextureBufferGL(session_, mirrorTexture_, &mirrorTextureId);

    glGenFramebuffers(1, &mirrorFBOID_);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBOID_);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mirrorTextureId, 0);
    glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
}


void OculusRenderer::initializeTracking() 
{
    ovr_SetTrackingOriginType(session_, ovrTrackingOrigin_FloorLevel);
    frameIndex_ = 0;
    sensorSampleTime_ = 0.0;
}


void OculusRenderer::initializeRectangleBuffers() 
{
    // Compute the useful part of the ZED image
    unsigned int widthFinal = bufferSize_.w / 2;
    float heightGL = 1.f;
    float widthGL = 1.f;
    if (captureWidth_ > 0.f) {
        unsigned int heightFinal = captureHeight_ * widthFinal / (float)captureWidth_;
        // Convert this size to OpenGL viewport's frame's coordinates
        heightGL = (heightFinal) / (float)(bufferSize_.h);
        widthGL = ((captureWidth_ * (heightFinal / (float)captureHeight_)) / (float)widthFinal);
    }
    else {
        std::cout << "WARNING: Video capture parameters got wrong values."
            "Default vertical and horizontal FOV are used."
            << std::endl;
    }

    // Compute the Horizontal Oculus' field of view with its parameters
    float ovrFovH = (atanf(hmdDesc_.DefaultEyeFov[0].LeftTan) + atanf(hmdDesc_.DefaultEyeFov[0].RightTan));
    // Compute the Vertical Oculus' field of view with its parameters
    float ovrFovV = (atanf(hmdDesc_.DefaultEyeFov[0].UpTan) + atanf(hmdDesc_.DefaultEyeFov[0].DownTan));

    // Compute the center of the optical lenses of the headset
    float offsetLensCenterX = ((atanf(hmdDesc_.DefaultEyeFov[0].LeftTan)) / ovrFovH) * 2.f - 1.f;
    float offsetLensCenterY = ((atanf(hmdDesc_.DefaultEyeFov[0].UpTan)) / ovrFovV) * 2.f - 1.f;

    // Create a rectangle with the computed coordinates and push it in GPU memory
    struct GLScreenCoordinates {
        float left, up, right, down;
    } screenCoord;

    screenCoord.up = heightGL + offsetLensCenterY;
    screenCoord.down = heightGL - offsetLensCenterY;
    screenCoord.right = widthGL * .75 + offsetLensCenterX;
    screenCoord.left = widthGL * .75 - offsetLensCenterX;

    float rectVertices[12] = { -screenCoord.left, -screenCoord.up, 0, screenCoord.right, -screenCoord.up, 0, screenCoord.right, screenCoord.down, 0, -screenCoord.left, screenCoord.down, 0 };
    glGenBuffers(1, &rectVBO_[0]);
    glBindBuffer(GL_ARRAY_BUFFER, rectVBO_[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(rectVertices), rectVertices, GL_STATIC_DRAW);

    float rectTexCoord[8] = { 0, 1, 1, 1, 1, 0, 0, 0 };
    glGenBuffers(1, &rectVBO_[1]);
    glBindBuffer(GL_ARRAY_BUFFER, rectVBO_[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(rectTexCoord), rectTexCoord, GL_STATIC_DRAW);

    unsigned int rectIndices[6] = { 0, 1, 2, 0, 2, 3 };
    glGenBuffers(1, &rectVBO_[2]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rectVBO_[2]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(rectIndices), rectIndices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void OculusRenderer::setupShaderAttributes() 
{
    // Enable the shader
    glUseProgram(shader_->getProgramId());
    
    // Bind the Vertex Buffer Objects of the rectangle that displays video capture images
    // vertices
    glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);
    glBindBuffer(GL_ARRAY_BUFFER, rectVBO_[0]);
    glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
    // indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rectVBO_[2]);
    // texture coordinates
    glEnableVertexAttribArray(Shader::ATTRIB_TEXTURE2D_POS);
    glBindBuffer(GL_ARRAY_BUFFER, rectVBO_[1]);
    glVertexAttribPointer(Shader::ATTRIB_TEXTURE2D_POS, 2, GL_FLOAT, GL_FALSE, 0, 0);
}


// ============================================================================
//                        Private Methods - Update
// ============================================================================

void OculusRenderer::grabFrame(VideoCaptureFrameBuffer& buffer) 
{

    // Get texture swap index where we must draw our frame
    GLuint curTexId;
    int curIndex;
    ovr_GetTextureSwapChainCurrentIndex(session_, textureChain_, &curIndex);
    ovr_GetTextureSwapChainBufferGL(session_, textureChain_, curIndex, &curTexId);

    // Call ovr_GetRenderDesc each frame to get the ovrEyeRenderDesc, as the returned values (e.g. HmdToEyeOffset) may change at runtime.
    hmdToEyeOffset_[ovrEye_Left] = ovr_GetRenderDesc(session_, ovrEye_Left, hmdDesc_.DefaultEyeFov[ovrEye_Left]).HmdToEyePose;
    hmdToEyeOffset_[ovrEye_Right] = ovr_GetRenderDesc(session_, ovrEye_Right, hmdDesc_.DefaultEyeFov[ovrEye_Right]).HmdToEyePose;

    // Get eye poses, feeding in correct IPD offset
    ovr_GetEyePoses2(session_, frameIndex_, ovrTrue, hmdToEyeOffset_, eyeRenderPose_, &sensorSampleTime_);

    // If the application is not focused, return early
    if (!isVisible_) {
        return;
    }

    // If successful grab a new capture image
    if (buffer.new_frame) {
        // Set the flag to false
        buffer.new_frame = false;

        BindCVMat2GLTexture(buffer.leftImage, captureTextureID_[0]);
        BindCVMat2GLTexture(buffer.rightImage, captureTextureID_[1]);

        // Bind the frame buffer
        glBindFramebuffer(GL_FRAMEBUFFER, fboID_);
        // Set its color layer 0 as the current swap texture
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, curTexId, 0);
        // Set its depth layer as our depth buffer
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthBuffID_, 0);
        // Clear the frame buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0, 0, 0, 1);

        // Render for each Oculus eye the equivalent video capture image
        for (int eye = 0; eye < 2; eye++) {
            // Set the left or right vertical half of the buffer as the viewport
            glViewport(eye == ovrEye_Left ? 0 : bufferSize_.w / 2, 0, bufferSize_.w / 2, bufferSize_.h);
            // Bind the left or right video capture image
            glBindTexture(GL_TEXTURE_2D, eye == ovrEye_Left ? captureTextureID_[ovrEye_Left] : captureTextureID_[ovrEye_Right]);
            // Bind the isLeft value
            glUniform1ui(glGetUniformLocation(shader_->getProgramId(), "isLeft"), eye == ovrEye_Left ? 1U : 0U);
            // Draw the video capture image
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        }
        // Avoids an error when calling SetAndClearRenderSurface during next iteration.
        // Without this, during the next while loop iteration SetAndClearRenderSurface
        // would bind a framebuffer with an invalid COLOR_ATTACHMENT0 because the texture ID
        // associated with COLOR_ATTACHMENT0 had been unlocked by calling wglDXUnlockObjectsNV.
        glBindFramebuffer(GL_FRAMEBUFFER, fboID_);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
        // Commit changes to the textures so they get picked up frame
        ovr_CommitTextureSwapChain(session_, textureChain_);
    }
    // Do not forget to increment the frameIndex!
    frameIndex_++;
}


void OculusRenderer::renderToOculus(VideoCaptureFrameBuffer& buffer) 
{
    /*
    Note: Even if we don't ask to refresh the framebuffer or if the Camera::grab()
            doesn't catch a new frame, we have to submit an image to the Oculus; it
            needs 75Hz refresh. Otherwise there will be jumbs, black frames and/or glitches
            in the headset.
    */
    ovrLayerEyeFov ld;
    ld.Header.Type = ovrLayerType_EyeFov;
    // Tell to the Oculus compositor that our texture origin is at the bottom left
    ld.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft; // Because OpenGL | Disable head tracking
    // Set the Oculus layer eye field of view for each view
    for (int eye = 0; eye < 2; ++eye) {
        // Set the color texture as the current swap texture
        ld.ColorTexture[eye] = textureChain_;
        // Set the viewport as the right or left vertical half part of the color texture
        ld.Viewport[eye] = OVR::Recti(eye == ovrEye_Left ? 0 : (bufferSize_.w / 2) + 200, 0, (bufferSize_.w / 2) - 200, bufferSize_.h);
        // Set the field of view
        ld.Fov[eye] = hmdDesc_.DefaultEyeFov[eye];
        // Set the pose matrix
        ld.RenderPose[eye] = eyeRenderPose_[eye];
    }

    ld.SensorSampleTime = sensorSampleTime_;

    ovrLayerHeader* layers = &ld.Header;
    // Submit the frame to the Oculus compositor
    // which will display the frame in the Oculus headset
    ovrErrorInfo errInf;
    ovrResult result = ovr_SubmitFrame(session_, frameIndex_, nullptr, &layers, 1);

    if (!OVR_SUCCESS(result)) {
        ovr_GetLastErrorInfo(&errInf);
        throw std::runtime_error(std::string("ERROR: failed to submit frame: ") + errInf.ErrorString);
    }

    if (result == ovrSuccess && !isVisible_) {
        std::cout << "The application is now shown in the headset." << std::endl;
    }

    isVisible_ = (result == ovrSuccess);

    // This is not really needed for this application but it may be useful for an more advanced application
    ovrSessionStatus sessionStatus;
    ovr_GetSessionStatus(session_, &sessionStatus);
    if (sessionStatus.ShouldRecenter) {
        std::cout << "Recenter Tracking asked by Session" << std::endl;
        ovr_RecenterTrackingOrigin(session_);
    }

    // Copy the frame to the mirror buffer which will be drawn in the SDL2 image
    glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBOID_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    GLint w = winWidth_;
    GLint h = winHeight_;
    glBlitFramebuffer(0, h, w, 0,
        0, 0, w, h,
        GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    // Swap the SDL2 window
    SDL_GL_SwapWindow(window_);
}
