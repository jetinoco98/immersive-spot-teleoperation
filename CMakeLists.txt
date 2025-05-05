cmake_minimum_required(VERSION 3.15)
project(immersive_spot_teleoperation LANGUAGES CXX CUDA)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Set the policy to avoid warning for the removed FindCUDA module
cmake_policy(SET CMP0146 NEW)

# System Libraries
find_package(OpenGL REQUIRED)
find_package(ZED REQUIRED)  # User must install manually
find_package(CUDAToolkit REQUIRED)  # User must install manually

# OpenCV binaries stored on the project external folder
set(OpenCV_DIR "${CMAKE_SOURCE_DIR}/external/opencv")
find_package(OpenCV REQUIRED)
set(OpenCV_LIB_DIRS "${CMAKE_SOURCE_DIR}/external/opencv/x64/vc16/lib")

# Create a shared library target
add_library(core_lib INTERFACE)

# Include directories
target_include_directories(core_lib INTERFACE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/external/zmq/include
    ${CMAKE_SOURCE_DIR}/external/glew/include
    ${CMAKE_SOURCE_DIR}/external/SDL2/include
    ${CMAKE_SOURCE_DIR}/external/ovr_sdk/LibOVR/Include
    ${OpenCV_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
)

# Link directories
target_link_directories(core_lib INTERFACE
    ${CMAKE_SOURCE_DIR}/external/zmq/lib
    ${CMAKE_SOURCE_DIR}/external/glew/lib/Release/x64
    ${CMAKE_SOURCE_DIR}/external/SDL2/lib/x64
    ${CMAKE_SOURCE_DIR}/external/ovr_sdk/LibOVR/Lib/Windows/x64/Release/VS2017
    ${OpenCV_LIB_DIRS}
)

# Link libraries
target_link_libraries(core_lib INTERFACE
    libzmq-v143-mt-4_3_6
    glew32
    SDL2
    OpenGL::GL
    LibOVR
    ${ZED_LIBRARIES}
    CUDA::cudart
    ${OpenCV_LIBS}
)

# Define the main executables
set(CMAKE_WIN32_EXECUTABLE FALSE) # To ensure it's treated as a console application
add_executable(${CMAKE_PROJECT_NAME} "src/main.cpp" "src/Shader.cpp")
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE core_lib)

# Define the test executables
add_executable(check_libraries test/check_libraries.cpp)
add_executable(hdm_test test/hdm_test.cpp)
target_link_libraries(check_libraries PRIVATE core_lib)
target_link_libraries(hdm_test PRIVATE core_lib)


# -- DLL File Copy --
# Copy all DLLs from the 'dlls' folder into the output directory after build
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${PROJECT_SOURCE_DIR}/external/dlls"
            "$<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>"
)

# -- JSON File Copy --
# Copy config.json from the source directory to the build directory
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            "${CMAKE_SOURCE_DIR}/config.json"
            "${CMAKE_BINARY_DIR}/Release/config.json"
) 