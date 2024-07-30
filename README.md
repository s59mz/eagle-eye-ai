# Eagle-Eye-AI

Eagle-Eye-AI is a project designed for the Kria KR260 board that enables AI-driven camera tracking and face detection. The project integrates custom hardware and software, including a RS-485 PMOD module for camera rotator control and ROS2 nodes for real-time processing and communication. Follow our comprehensive guide on [Hackster.io](https://www.hackster.io/matjaz4) to build, test, and deploy the system, transforming your camera into an intelligent, autonomous tracking device.

## Requirements

1. **Kria KR260 Board**: Ensure that your KRIA™ KR260 board has the official Ubuntu image installed and Docker set up. The board should be prepared for running official demo applications from AMD, such as the **Smartcam demo** application.

2. **IP Camera**: You will need an IP camera that supports RTSP streaming with a resolution of 1920x1080. The camera should be connected to the same local network as the Kria board. A recommended camera is the [SIMICAM 4k Video Cam](https://a.aliexpress.com/_EznpRub) or similar.

3. **Pan-Tilt Rotator**: A Pan-Tilt Camera Rotator that supports RS-485 and the Pelco-P/D protocol is required for rotating the camera. A recommended rotator is the [PTZ Rotator](https://a.aliexpress.com/_EvhGQMB).

4. **RS-485 PMOD Module**: Required for camera rotator control. The Camera Rotator should support the Pelco-D protocol for Pan-Tilt through RS-485 interface. The module can be found [here](https://github.com/s59mz/kicad-pmod_rs485).

5. **Network Connection**: Connect the Ethernet cable to your local network with DHCP enabled.

## Getting the Application Package

1. **Clone the Repository**:

    ```bash
    git clone https://github.com/s59mz/eagle-eye-ai.git
    cd eagle-eye-ai
    ```

## Install Firmware Binaries

1. Install the firmware binaries:

    ```bash
    cp fpga-firmware/firmware-kr260-eagle-eye.deb /tmp
    sudo apt install /tmp/firmware-kr260-eagle-eye.deb
    ```

2. Dynamically load the firmware package:

    * Switch to the kr260-eagle-eye platform:

      ```bash
      sudo xmutil unloadapp
      sudo xmutil loadapp kr260-eagle-eye
      ```
    
    * Show the list and status of available acceleration platforms:

      ```bash
      sudo xmutil listapps
      ```

3. Disable the desktop environment:

      ```bash
      sudo xmutil desktop_disable
      ```

## Building the Docker Image

1. **Update the RTSP IP Camera URL**:

    Edit the `run_eagle_eye_ai.sh` script file in the `scripts` directory and update the `default_camera_url` parameter:

    ```bash
    vi scripts/run_eagle_eye_ai.sh

    # Update line #18 with your IP camera URL:
    default_camera_url="rtsp://192.168.1.11:554/stream1"
    ```

2. **Build the Docker Image**:

    The build process will take about 8 minutes on the Kria board. This cannot be built on a host PC.

    ```bash
    ./build.sh
    ```

    For a visual guide on building and installing the Eagle-Eye-AI application, watch this [YouTube video](https://www.youtube.com/watch?v=w_0K5YZrkO0).

    [![Building and Installing the Eagle Eye AI Application](https://img.youtube.com/vi/w_0K5YZrkO0/hqdefault.jpg)](https://www.youtube.com/watch?v=w_0K5YZrkO0)

## Launching the Docker Image

1. **Launch the Docker Image**:

    ```bash
    ./run.sh
    ```

    This will start the Eagle-Eye-AI Docker image in a new container:

    ```bash
    root@xlnx-docker/#
    ```

2. **Optional: Run an Empty ROS2 Docker Image**:

    To test your own ROS2/GStreamer applications:

    ```bash
    ./ros2_humble_run.sh
    ```

3. **Manage Docker Storage**:

    To remove the existing container if storage is limited:

    ```bash
    docker rmi --force eagle-eye-ai
    ```

## Running the Application

1. In the running Eagle-Eye-AI Docker container:

    * Launch the application:

      ```bash
      ./run_eagle_eye_ai.sh
      ```

      You should see the camera’s captured images on the monitor connected to the board. When a face is detected, a blue box will appear around it, tracking the face as it moves. The camera rotator will also adjust to keep the detected face centered on the screen.

    * Press `Ctrl-C` to exit.

    * To change the RTSP IP camera URL, run the startup script with the new URL:

      ```bash
      ./run_eagle_eye_ai.sh rtsp://192.168.1.20:554/stream2
      ```

    For a visual guide on starting the Eagle-Eye-AI application, watch this [YouTube video](https://www.youtube.com/watch?v=IakoRX5yPNo).

    [![Starting the Eagle Eye AI Application](https://img.youtube.com/vi/IakoRX5yPNo/hqdefault.jpg)](https://www.youtube.com/watch?v=IakoRX5yPNo)

## License

This project is licensed under the GPL-3.0. See the LICENSE file for details.

For further information or support, please refer to the project documentation on [Hackster.io](https://www.hackster.io/matjaz4).
