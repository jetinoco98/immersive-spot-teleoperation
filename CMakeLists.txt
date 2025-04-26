cmake_minimum_required(VERSION 3.15)
project(immersive_spot_teleoperation LANGUAGES CXX CUDA)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Set the policy to avoid warning for the removed FindCUDA module
cmake_policy(SET CMP0146 NEW)

# Libraries that are part of the system
find_package(OpenGL REQUIRED)

# Libraries that the user needs to manually install
find_package(ZED REQUIRED)
set(CUDA_DIR "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v12.8")
find_package(CUDAToolkit REQUIRED)

# Full library stored on the project itself
set(OpenCV_DIR "${CMAKE_SOURCE_DIR}/external/opencv/build")
find_package(OpenCV REQUIRED)
set(OpenCV_LIB_DIRS "${CMAKE_SOURCE_DIR}/external/opencv/build/x64/vc16/lib")


# Include headers
include_directories(
    ${CMAKE_SOURCE_DIR}/external/zmq/include
    ${CMAKE_SOURCE_DIR}/external/glew/include
    ${CMAKE_SOURCE_DIR}/external/SDL2/include
    ${CMAKE_SOURCE_DIR}/external/ovr_sdk/LibOVR/Include
    ${OpenCV_INCLUDE_DIRS}
)


# Link libraries
link_directories(
    ${CMAKE_SOURCE_DIR}/external/zmq/lib
    ${CMAKE_SOURCE_DIR}/external/glew/lib/Release/x64
    ${CMAKE_SOURCE_DIR}/external/SDL2/lib/x64
    ${CMAKE_SOURCE_DIR}/external/ovr_sdk/LibOVR/Lib/Windows/x64/Release/VS2017
    ${OpenCV_LIB_DIRS}
)

# Define your executable
add_executable(main_test src/main_test.cpp)


target_link_libraries(main_test PRIVATE
    libzmq-v143-mt-4_3_6
    glew32
    SDL2
    OpenGL::GL
    LibOVR
    ${ZED_LIBRARIES}
    CUDA::cudart
    ${OpenCV_LIBS}
)

# Include ZED headers and libraries
target_include_directories(main_test PRIVATE ${ZED_INCLUDE_DIRS})


# Set the subsystem to Console (for console apps)
set_target_properties(main_test PROPERTIES LINK_FLAGS "/SUBSYSTEM:CONSOLE")


# -- General automated steps --

# Define the target DLL output folder (Release or Debug)
set(DLL_OUTPUT_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

# Copy all DLLs from the 'dlls' folder into the output directory after build
add_custom_command(TARGET main_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${PROJECT_SOURCE_DIR}/external/dlls"
            "$<TARGET_FILE_DIR:main_test>"
)