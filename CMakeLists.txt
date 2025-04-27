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

# OpenCV binaries stored on the project itself
set(OpenCV_DIR "${CMAKE_SOURCE_DIR}/external/opencv/build")
find_package(OpenCV REQUIRED)
set(OpenCV_LIB_DIRS "${CMAKE_SOURCE_DIR}/external/opencv/build/x64/vc16/lib")

# Define the main executable
set(CMAKE_WIN32_EXECUTABLE FALSE) # To ensure it's treated as a console application
set(SRC_FOLDER src)
file(GLOB_RECURSE SRC_FILES "${SRC_FOLDER}/*.cpp")
add_executable(${CMAKE_PROJECT_NAME} ${SRC_FILES})

# Include directories
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/external/zmq/include
    ${CMAKE_SOURCE_DIR}/external/glew/include
    ${CMAKE_SOURCE_DIR}/external/SDL2/include
    ${CMAKE_SOURCE_DIR}/external/ovr_sdk/LibOVR/Include
    ${OpenCV_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}  # ZED-specific includes
)

# Link directories
target_link_directories(${CMAKE_PROJECT_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}/external/zmq/lib
    ${CMAKE_SOURCE_DIR}/external/glew/lib/Release/x64
    ${CMAKE_SOURCE_DIR}/external/SDL2/lib/x64
    ${CMAKE_SOURCE_DIR}/external/ovr_sdk/LibOVR/Lib/Windows/x64/Release/VS2017
    ${OpenCV_LIB_DIRS}
)

# Link libraries
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    libzmq-v143-mt-4_3_6
    glew32
    SDL2
    OpenGL::GL
    LibOVR
    ${ZED_LIBRARIES}
    CUDA::cudart
    ${OpenCV_LIBS}
)


# -- TEMPORARY: Executable for tests --
add_executable(project_test test/publisher.cpp)

target_include_directories(project_test PUBLIC
    ${CMAKE_SOURCE_DIR}/external/zmq/include
)

target_link_directories(project_test PUBLIC
    ${CMAKE_SOURCE_DIR}/external/zmq/lib
)

target_link_libraries(project_test PRIVATE
    libzmq-v143-mt-4_3_6
)


# -- DLL Copy Automation --
# Define the target DLL output folder (Release or Debug)
set(DLL_OUTPUT_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

# Copy all DLLs from the 'dlls' folder into the output directory after build
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${PROJECT_SOURCE_DIR}/external/dlls"
            "$<TARGET_FILE_DIR:${CMAKE_PROJECT_NAME}>"
)