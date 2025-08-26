#
# Eagle-Eye-AI
# Smart Following Camera with Face Recognition
#   for Kria KR260 Board
#
# Created by: Matjaz Zibert S59MZ - July 2024
#
# Main Startup script for the application
#   - Starts the eagle-eye-ai Docker image
#
# Design based on Kria KV260 Smartcam Demo App by AMD
#
# Hackster.io Project link:
#     https://www.hackster.io/matjaz4
#


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
-v /run:/run \
-it eagle-eye-ai:vvas bash
