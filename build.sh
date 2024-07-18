#!/bin/bash
#
# Eagle-Eye-AI 
#  Docker Build Script for Kria KR260 Board
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


# Define the names of the images
ROS_IMAGE="ros2-humble:1.0"
APP_IMAGE="eagle-eye-ai:1.0"

# Define the names of dockerfiles
ROS_DOCKERFILE="ros2-humble-docker"
APP_DOCKERFILE="eagle-eye-docker"

# Check if ros-image exists
if ! docker image inspect $ROS_IMAGE > /dev/null 2>&1; then
    echo "$ROS_IMAGE does not exist. Building $ROS_IMAGE..."
    docker build --build-arg BUILD_DATE="$(date -u +'%Y/%m/%d %H:%M')" -f $ROS_DOCKERFILE . -t $ROS_IMAGE
else
  echo "$ROS_IMAGE already exists."
fi

# Build app-image
if docker image inspect $ROS_IMAGE > /dev/null 2>&1; then
    echo "Building $APP_IMAGE..."
    docker build --build-arg BUILD_DATE="$(date -u +'%Y/%m/%d %H:%M')" -f $APP_DOCKERFILE . -t $APP_IMAGE
else
  echo "ERROR: Can't build the $ROS_IMAGE image"
fi

if ! [ docker image inspect $APP_IMAGE > /dev/null 2>&1 ] && ! [ docker image inspect $ROS_IMAGE > /dev/null 2>&1 ] ; then
  echo "Both $ROS_IMAGE and $APP_IMAGE are now on Kria board."
else
  echo "ERROR: Can't build the $APP_IMAGE image"
fi
