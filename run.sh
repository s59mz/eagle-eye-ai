#
# Eagle-Eye-AI
# Smart Following Camera with Face Detection
#   for Kria KR260 Board
#
# Created by: Matjaz Zibert S59MZ - August 2025
#
# Main Startup script for the application
#   - Starts the eagle-eye-ai Docker image
#
# Design based on Kria KV260 Smartcam Demo App by AMD
#
# Hackster.io Project link:
#     https://www.hackster.io/matjaz4/eagleeye-ai-smart-following-camera-with-face-recognition-1b0f65
#

APP_IMAGE="eagle-eye-ai:3.5"

if  docker image inspect $APP_IMAGE > /dev/null 2>&1 ; then

    docker run \
    --rm \
    --net=host \
    --env="DISPLAY" \
    -h "xlnx-docker" \
    --env="XDG_SESSION_TYPE" \
    --privileged \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    -v /tmp:/tmp \
    -v /dev:/dev \
    -v /sys:/sys \
    -v /etc/vart.conf:/etc/vart.conf \
    -v /lib/firmware/xilinx:/lib/firmware/xilinx \
    -v $(pwd)/ros2_ws:/root/ros2_ws \
    -v $(pwd)/config/facedetect:/opt/xilinx/kr260-eagle-eye/share/vvas/facedetect \
    -v $(pwd)/models:/opt/xilinx/kr260-eagle-eye/share/vitis_ai_library/models \
    -it $APP_IMAGE bash

else
    echo ""
    echo "!!! The application docker image \"$APP_IMAGE\" hasn't been built yet. Type \"./build.sh\" to build it.";
    echo ""
fi

