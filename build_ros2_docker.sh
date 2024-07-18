docker build --build-arg BUILD_DATE="$(date -u +'%Y/%m/%d %H:%M')" -f ros2-humble-docker . -t ros2-humble:1.0
