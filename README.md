# Eagle-Eye-AI

Eagle-Eye-AI is a project designed for the Kria KR260 board that enables AI-driven camera tracking and face detection. The project integrates custom hardware and software, including a PMOD RS-485 module for camera rotator control and ROS2 nodes for real-time processing and communication. Follow our comprehensive guide on [Hackster](https://www.hackster.io/matjaz4) to build, test, and deploy the system, transforming your camera into an intelligent, autonomous tracking device.

# Setting up the Board

1. Get the SD Card Image from [Boot Image Site](https://ubuntu.com/download/amd-xilinx) and follow the instructions in UG1089 to burn the SD card. And install the SD card to J11.

2. Hardware Setup:

    * Monitor:
    
      Before booting, connect a 1080P/4K monitor to the board via either DP or HDMI port.

      4K monitor is preferred to demonstrate at the maximum supported resolution.

    * UART/JTAG interface:
    
      For interacting and seeing boot-time information, connect a USB debugger to the J4.
    
    * You should use a RTSP webcam as an input device.
    
      The application requires an RTSP webcam as the mandatory video input device. The recommended and tested resolution is **1920x1080**.

      Recommended webcam is the [SIMICAM 4k Video Cam](https://a.aliexpress.com/_EznpRub).

    * Camera Rotator:

      Mount the camera to a Pan-Tilt Camera Rotator that supports RS-485 and Pelco-P/D protocol for rotating the camera.

      Recommended camera rotator is the [PTZ Rotator](https://a.aliexpress.com/_EvhGQMB).
      
    * Network connection:
    
      Connect the Ethernet cable to your local network with DHCP enabled.

    * [RS-485 PMOD module](https://github.com/s59mz/kicad-pmod_rs485) as camera rotator control 

      The Camera Rotator should support the Pelco-D protocol for Pan-Tilt through RS-485 interface.
      
      In eagle-eye application supports only RTSP camera input mode.

      Eagle-Eye application does not support speakers.
    
3. Software Preparation:

    You will use a PC having network access to the board as the SSH client machine.

    Make sure that the PC and the KR260 Robotic AI Starter Kit are on the same subnet mask.

4. Power on the board, and booting your Starter Kit (Ubuntu):

   * Follow the instruction from the page below to boot linux

  	https://www.xilinx.com/products/som/kria/kr260-robotics-starter-kit/kr260-getting-started/booting-your-starter-kit.html

5. Set System Timezone and locale:

    * Set timezone

       ```bash
		sudo timedatectl set-ntp true
		sudo timedatectl set-timezone America/Los_Angeles
		timedatectl
       ```
	
	* Set locale

       ```bash
		sudo locale-gen en_US en_US.UTF-8
		sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
		export LANG=en_US.UTF-8
		locale
       ```

6. Update Bootfirmware

    The SOM Starter Kits have factory pre-programmed boot firmware that is installed and maintained in the SOM QSPI device. Update the Boot firmware in the SOM QSPI device to '2022.1 Boot FW' Image.

    Follow the link below to obtain Boot firmware binary and instructions to update QSPI image using xmutil, after linux boot.  

    https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/1641152513/Kria+K26+SOM#Boot-Firmware-Updates

7. Install Docker from [here](https://docs.docker.com/engine/install/ubuntu/).

9. Enable your user to properly use the docker commands without using sudo for every command. 

    ```bash
    sudo groupadd docker
    sudo usermod -a -G docker  $USER
	```

# Geting the Application Package

1. Get the latest kr260-eagle-eye package:

	* Get the Eagle-Eye-AI git repository from GitHub

    ```bash
    git clone https://github.com/s59mz/eagle-eye-ai.git
    ```

	* Enter to working directory

    ```bash
    cd eagle-eye-ai
    ```

# Install Firmware Binaries.

1. The following procedure would instal the .bit.bin, .xclbin, .dtbo and .json files to the Kria board's "/lib/firmware/xilinx/kr260-eagle-eye" file location:

    ```bash
    cp fpga-firmware/firmware-kr260-eagle-eye.deb /tmp
    sudo apt install /tmp/firmware-kr260-eagle-eye.deb
    ```

2. Dynamically load the firmware package:

    The firmware consist of bitstream, binary header and device tree overlay (dtbo) file. The firmware is loaded dynamically on user request once Linux is fully booted. The xmutil utility can be used for that purpose.

    * Disable the desktop environment:

       ```bash
       sudo xmutil      desktop_disable
       ```

       After running the application, the desktop environment can be enable again with:

       ```bash
       sudo xmutil      desktop_enable
       ```

    * Show the list and status of available acceleration platforms :

       ```bash
      sudo xmutil listapps
        ```

    * Switch to a different platform for different Application:

       When there's already another accelerator/firmware being activated, unload it first, then switch to kr260-eagle-eye.

       ```bash
      sudo xmutil unloadapp
      sudo xmutil loadapp kr260-eagle-eye
       ```

# Building the Docker Image

1. Update the RTSP IP camera URL before start to build your application (you can also change this later too)

    * Edit the "run_eagle_eye_ai.sh" script file in "scripts" directory and update the "default_camera_url" parameter:

    ```bash
    vi scripts/run_eagle_eye_ai.sh

    # uptade line #18 with your IP camera URL:
    default_camera_url="rtsp://192.168.1.11:554/stream1"
    ```

2. Build the docker image for eagle-eye-ai using the below command. The building process will last about 8 min on Kria board. Cannot be build on host PC.

    ```bash
    ./build.sh
    ```

# Launching the Docker Image

1. Launch the docker image using the below command

    ```bash
    ./run.sh
    ```

    It will launch the eagle-eye docker image in a new container

    ```bash
    root@xlnx-docker/#
    ```

2. Optionally, you can run an empty ROS2 docker image with preinstalled GStreamer and VVAS support for testing your own ROS2/Gstreamer applications by command below:

    ```bash
    ./ros2_humble_run.sh
    ```

3. The storage volume on the SD card is limited with multiple dockers. You can use following command to remove the existing container.

    ```bash
    docker rmi --force eagle-eye-ai
    ```

# Running the Application

1.  In the running Eagle-Eye-AI docker container 

    * Launch the folowing command:

    ```bash
    ./run_eagle_eye_ai.sh
    ```

    You should see the camera’s captured images on the monitor connected to the board. When a face is detected, a blue box will appear around it, tracking the face as it moves. The camera rotator should also move to keep the closest detected face in the center of the screen, with the blue box remaining around the face.

    * Press Ctrl-C for exit

    * To change the RTSP IP camera URL, run the startup script with camera's URL as parameter:
      
    ```bash
    ./run_eagle_eye_ai.sh rtsp://192.168.1.20:554/stream2
    ```

    
