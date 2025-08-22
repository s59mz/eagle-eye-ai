#!/bin/bash
#
# Eagle-Eye-AI
# Smart Following Camera with Face Recognition
#   for Kria KR260 Board
#
# Created by: Matjaz Zibert S59MZ - July 2024
#
# Build script for two docker images:
#   - ROS2 Image - ROS2 and GStreamer based on Xilinx kria-runtime:2022.1 image
#   - APP Image - Final application image based on the ROS2 Image
#
# Design based on Kria KV260 Smartcam Demo App by AMD
#
# Hackster.io Project link:
#     https://www.hackster.io/matjaz4
#


# Define the names of the images
KRIA_RUNTIME_IMAGE="kria-runtime:3.5.0"
KRIA_DEV_IMAGE="kria-developer:3.5.0"
ROS_IMAGE="ros2-humble:3.5.0"
APP_IMAGE="eagle-eye-ai:3.5.0"

# Define the names of dockerfiles
KRIA_RUNTIME_DOCKERFILE="kria-runtime"
KRIA_DEVELOPER_DOCKERFILE="kria-developer"
ROS_DOCKERFILE="ros2-humble-docker"
APP_DOCKERFILE="eagle-eye-docker"

# Check if kria-runtime image exists
if ! docker image inspect $KRIA_RUNTIME_IMAGE > /dev/null 2>&1; then
    echo "$KRIA_RUNTIME_IMAGE does not exist. Building $KRIA_RUNTIME_IMAGE..."
    docker build --network host -f $KRIA_RUNTIME_DOCKERFILE . -t $KRIA_RUNTIME_IMAGE
else
  echo "$KRIA_RUNTIME_IMAGE already exists."
fi

# Check if kria-developer image exists
if ! docker image inspect $KRIA_DEV_IMAGE > /dev/null 2>&1; then
    echo "$KRIA_DEV_IMAGE does not exist. Building $KRIA_DEV_IMAGE..."
    docker build --network host -f $KRIA_DEV_DOCKERFILE . -t $KRIA_DEV_IMAGE
else
  echo "$KRIA_DEV_IMAGE already exists."
fi

# Check if ros-image exists
if ! docker image inspect $ROS_IMAGE > /dev/null 2>&1; then
    echo "$ROS_IMAGE does not exist. Building $ROS_IMAGE..."
    docker build --network host --build-arg BUILD_DATE="$(date -u +'%Y/%m/%d %H:%M')" -f $ROS_DOCKERFILE . -t $ROS_IMAGE
else
  echo "$ROS_IMAGE already exists."
fi

# Build app-image
if docker image inspect $ROS_IMAGE > /dev/null 2>&1; then
    echo "Building $APP_IMAGE..."
    docker build --network host --build-arg BUILD_DATE="$(date -u +'%Y/%m/%d %H:%M')" -f $APP_DOCKERFILE . -t $APP_IMAGE
else
  echo "ERROR: Can't build the $ROS_IMAGE image"
fi

if ! [ docker image inspect $APP_IMAGE > /dev/null 2>&1 ] && ! [ docker image inspect $ROS_IMAGE > /dev/null 2>&1 ] ; then
  echo "Both $ROS_IMAGE and $APP_IMAGE are now on Kria board."
else
  echo "ERROR: Can't build the $APP_IMAGE image"
fi
