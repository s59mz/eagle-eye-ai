
# Setting up the Board

1. Get the SD Card Image from [Boot Image Site](https://ubuntu.com/download/amd-xilinx) and follow the instructions in UG1089 to burn the SD card. And install the SD card to J11.

2. Hardware Setup:

    * Monitor:
    
      Before booting, connect a 1080P/4K monitor to the board via either DP or HDMI port.

      4K monitor is preferred to demonstrate at the maximum supported resolution.

    * UART/JTAG interface:
    
      For interacting and seeing boot-time information, connect a USB debugger to the J4.
    
    * You should use a RTSP webcam as an input device.
    
      The RTSP webcam is mandatory video input device supported in the application.

      Recommended webcam is the [SIMICAM 4k Video Cam](https://a.aliexpress.com/_EznpRub).

    * Camera Rotator:

      Mount the camera to a Pan-Tilt Camera Rotator that supports RS-485 and Pelco-P/D protocol for rotating the camera

      Recommended camera rotator is the [PTZ Rotator](https://a.aliexpress.com/_EvhGQMB).
      
    * Network connection:
    
      Connect the Ethernet cable to your local network with DHCP enabled.

    * RS-485 Pmod module as camera rotator control 

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

8. Get the latest kr260-eagle-eye firmware package:

	* Get the Eagle-Eye-AI gir repository from GitHub

    ```bash
    git clone https://github.com/s59mz/eagle-eye-ai.git
    ```

	* Enter to that directory

    ```bash
    cd eagle-eye-ai
    ```

	* Install firmware binaries

    ```bash
    sudo apt install ./fpga-firmware/firmware-kr260-eagle-eye.deb
    ```

9. Dynamically load the application package:

    The firmware consist of bitstream, device tree overlay (dtbo) file. The firmware is loaded dynamically on user request once Linux is fully booted. The xmutil utility can be used for that purpose.

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

10. Enable your user to properly use the docker commands without using sudo for every command. 

    ```bash
    sudo groupadd docker
    sudo usermod -a -G docker  $USER
	```

11. Build the docker image for eagle-eye-ai using the below command. The building process will last about 8 min on Kria board. Cannot be build on host PC.

    ```bash
    ./build.sh
    ```

12. Launch the docker using the below command

    ```bash
    ./run.sh
    ```

    It will launch the eagle-eye image in a new container

    ```bash
    root@xlnx-docker/#
    ```

    Optionally, you can run an empty ROS2 docker with GStreamer and VVAS support for testing your own ROS2 applications by command below

    ```bash
    ./ros2_humble_run.sh
    ```

14. The storage volume on the SD card is limited with multiple dockers. You can use following command to remove the existing container.

    ```bash
    docker rmi --force eagle-eye
    ```

# Run the Application

1.  Run the command: 

    ```bash
    ./run_eagle_eye_ai.sh
    ```

    You should be able to see the images the camera is capturing on the monitor connected to the board, and when there's face captured by the camera, there should be blue box drawn around the face, and the box should follow the movement of the face. The camera rotator should start moving, so the camera follows the closest detected face and the blue box with the face should stays in the middle of the screen.


