# Eagle-Eye-AI

Eagle-Eye-AI is a project designed for the Kria KR260 board that enables AI-driven camera tracking and face detection. The project integrates custom hardware and software, including a PMOD RS-485 module for camera rotator control and ROS2 nodes for real-time processing and communication. Follow our comprehensive guide on [Hackster.io](https://www.hackster.io/matjaz4) to build, test, and deploy the system, transforming your camera into an intelligent, autonomous tracking device.

## Requirements

Ensure that your KRIA™ KR260 board has the official Ubuntu image installed and Docker set up. The board should also be prepared for running official demo applications from AMD, such as the Smartcam demo application.

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

    * Disable the desktop environment:

      ```bash
      sudo xmutil desktop_disable
      ```

      Enable it again after running the application with:

      ```bash
      sudo xmutil desktop_enable
      ```

    * Show the list and status of available acceleration platforms:

      ```bash
      sudo xmutil listapps
      ```

    * Switch to the kr260-eagle-eye platform:

      ```bash
      sudo xmutil unloadapp
      sudo xmutil loadapp kr260-eagle-eye
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

This project is licensed under the [Your License Here]. See the LICENSE file for details.

For further information or support, please refer to the project documentation or contact the repository maintainers.
