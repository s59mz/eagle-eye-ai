#!/bin/bash
#
# Eagle-Eye-AI
#  Startup Script tomlaunch the app  Kria KR260 Board
#
# Created by: Matjaz Zibert S59MZ - July 2024
#
# Design based on Kria KV260 Smartcam Demo App
#
# Original Smartcam link:
#     https://github.com/Xilinx/smartcam/tree/xlnx_rel_v2022.1
#     https://github.com/Xilinx/kria-docker/tree/xlnx_rel_v2022.1
#
#   NOTE: This script runs on Kria board only


# Default camera_url
default_camera_url="rtsp://192.168.1.11:554/stream1"

# Check if a camera_url parameter is passed
if [ -z "$1" ]; then
    # No parameter passed, use the default camera_url
    camera_url=$default_camera_url
else
    # Parameter passed, use it as camera_url
    camera_url=$1
fi

# Source the ROS2 setup script
source install/setup.bash

# Start the ROS2 launch file with the specified camera_url
ros2 launch eagle_eye_bringup follow_cam.launch.xml camera_url:=$camera_url

