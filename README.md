## Design and User Evaluation of an Immersive Teleoperation System for a Quadruped Robot

This repository contains a modular software architecture designed for immersive teleoperation of the Boston Dynamics Spot robot using the Meta Quest 2.


**Authors:** 
  - Ali Yousefi, ali.yousefi@edu.unige.it
  - Carmine Tommaso Recchiuto, carmine.recchiuto@dibris.unige.it
  - Antonio Sgorbissa, antonio.sgorbissa@unige.it
    
©2025 RICE Lab - DIBRIS, University of Genova
<p align="left">
<img src="https://github.com/user-attachments/assets/0fdac2aa-7100-4caa-9191-df72cb55c8be" width="150" title="rice_logo">
</p>

### Package Description
The package includes source code developed for the following functionalities:

1. Control Algorithm: Implements the control logic for the robot (``scripts/spot_client/spot_interface.py``).
2. Sensor Measurements and Control Commands: Handles robot sensor data and applies control commands (``scripts/spot_client/spot_controller.py``).
3. Stereo Image Capture: Captures stereo images from the ZED camera (``scripts/spot_client/zed_interface.py``).
4. Robot Control and Image Compression: Manages robot control and compresses images before transmission using a GStreamer pipeline (``scripts/spot_client/spot_client.py``).
5. HMD Rendering and Command Reading: Renders stereo images on the head-mounted display (HMD) and reads control commands (``src/main.cpp``).
6. Control Command Transmission: Sends the measured control commands to the robot (``scripts/oculus_client.pt``).

The last two source codes run on the user's PC (Windows), while the others run on the Jetson board (Ubuntu) mounted on the robot.

### Dependencies
**The required software on the user's PC are the following:**
- Windows 64 bits
- python (3.7.0 or later)
- CMake 
- Visual Studio 2022
- [Oculus SDK](https://developer.oculus.com/downloads/package/oculus-sdk-for-windows/) (1.17 or later)
- [CUDA](https://developer.nvidia.com/cuda-downloads).
- [GLEW](https://glew.sourceforge.net/) included in the ZED SDK dependencies folder
- [SDL](https://github.com/libsdl-org/SDL/releases/tag/release-2.30.1)
- [GStreamer](https://gstreamer.freedesktop.org/documentation/installing/on-windows.html?gi-language=missing:%20GSTREAMER_LIBRARY%20GSTREAMER_BASE_LIBRARY%20GSTREAMER_BASE_INCLUDE_DIR)
- [OpenCV](https://docs.opencv.org/4.x/d3/d52/tutorial_windows_install.html)
- [paho-mqtt](https://pypi.org/project/paho-mqtt/)

**On the Jetson board:**
- Ubuntu 18.04
- python (3.7.0 or later)
- CMake
- [Spot SDK](https://dev.bostondynamics.com/)
- [ZED SDK 3.x](https://www.stereolabs.com/developers) 
- [ZED Python API](https://www.stereolabs.com/docs/app-development/python/install)
- [Gstreamer](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html?gi-language=missing:%20GSTREAMER_LIBRARY%20GSTREAMER_BASE_LIBRARY%20GSTREAMER_BASE_INCLUDE_DIR)
- [OpenCV](https://docs.opencv.org/4.x/d2/de6/tutorial_py_setup_in_ubuntu.html)
- [do-mpc](https://www.do-mpc.com/en/latest/installation.html)
- [paho-mqtt](https://pypi.org/project/paho-mqtt/)

**On the cloud server:**
Create a Virtual Machine on a cloud server e.g., [Google Cloud](https://cloud.google.com/gcp?utm_source=google&utm_medium=cpc&utm_campaign=emea-it-all-en-bkws-all-all-trial-e-gcp-1707574&utm_content=text-ad-none-any-DEV_c-CRE_500236788708-ADGP_Hybrid+%7C+BKWS+-+EXA+%7C+Txt+-+GCP+-+General+-+v1-KWID_43700060384861753-kwd-6458750523-userloc_1008337&utm_term=KW_google%20cloud-NET_g-PLAC_&&gad_source=1&gclid=CjwKCAiAkc28BhB0EiwAM001TZVxUwbj72gOj4Y6C4xPtYfWtdJU1TFi5W-UiXgAR-4iRHT6YesYVRoCoVMQAvD_BwE&gclsrc=aw.ds), or [Microsoft Azure](https://azure.microsoft.com/en-us/pricing/purchase-options/azure-account/search?icid=free-search&ef_id=_k_CjwKCAiAkc28BhB0EiwAM001TbAAdZrGIjKV1fpHcYiFBH7cSAsD0j858p8zGkIlAga3w0IMXil1bRoCnxMQAvD_BwE_k_&OCID=AIDcmmy6frl1tq_SEM__k_CjwKCAiAkc28BhB0EiwAM001TbAAdZrGIjKV1fpHcYiFBH7cSAsD0j858p8zGkIlAga3w0IMXil1bRoCnxMQAvD_BwE_k_&gad_source=1&gclid=CjwKCAiAkc28BhB0EiwAM001TbAAdZrGIjKV1fpHcYiFBH7cSAsD0j858p8zGkIlAga3w0IMXil1bRoCnxMQAvD_BwE) and get the following dependencies:
- [Mediamtx](https://github.com/bluenviron/mediamtx#corrupted-frames)
- [Mosquitto](https://mosquitto.org/download/)

### Build
**On the user's PC:**
Clone the source files from the repositoy and follow the instructions below: 
1. Create a folder called "build" in the root folder
2. Open cmake-gui and select the source and build folders
3. Generate the Visual Studio Win64 solution
4. Open the resulting solution and change configuration to Release. You may have to modify the path of the dependencies to match your configuration
5. Build solution
Build OpenCV with GStreamer ([Tutorial](https://galaktyk.medium.com/how-to-build-opencv-with-gstreamer-b11668fa09c)).

**On the Jetson:**
The software does not require build, just clone the scripts files in the spot_clinet folder from the repository, but you have to build OpenCV with GStreamer on the Jetson as well ([Tutorial](https://galaktyk.medium.com/how-to-build-opencv-with-gstreamer-b11668fa09c)).

### Usage
**On the robot side (Linux/Jetson):** Run the ``spot_client.py`` script as follows:
```
python spot_client.py <cloud-server-public-ip> ZED
```
**On the user side (Windows):** Run the ''ZED_Stereo_Passthrough.exe'' in a terminal as it follows:
```
'./ZED Stereo Passthrough.exe' <cloud-server-public-ip> ZED
```
**On the cloud server:** Run the following commands:
```
sudo systemctl start mosqitto
```
```
./mediamtx
```

### System Hypothesis and Future Work
For future work, we aim to address the limitations of the current system and enhance its functionality. Planned improvements include:

- **Semi-Autonomous Control Logic:** Incorporate advanced algorithms to enable semi-autonomous navigation and task execution, reducing the cognitive load on the operator.

- **Enhanced Perception Capabilities:** Integrate additional sensors such as LIDAR or thermal cameras to improve environmental awareness and support complex tasks in challenging scenarios.

- **Real-Time Feedback Optimization:** Optimize latency in image and control data transmission to enhance real-time responsiveness, particularly in low-bandwidth network conditions.

- **Robust Communication Framework:** Implement fail-safe mechanisms and adaptive communication protocols to maintain system reliability in varying network conditions.

- **User-Centered Design Improvements:** Conduct user evaluations to refine the system interface, improving usability and immersion in teleoperation tasks.

- **Augmented Reality Integration:** Explore the use of augmented reality (AR) overlays on the HMD to provide contextual information such as obstacle warnings, path planning, or task-specific guidance.

- **Expanding Robot Compatibility:** Extend the system to support other robot platforms, making it a versatile solution for diverse robotic teleoperation applications.

By addressing these objectives, we aim to make the teleoperation system more efficient, reliable, and user-friendly, enabling a broader range of applications in fields such as search and rescue, inspection, and industrial automation.


