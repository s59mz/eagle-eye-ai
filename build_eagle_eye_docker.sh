docker build --build-arg BUILD_DATE="$(date -u +'%Y/%m/%d %H:%M')" -f eagle-eye-docker . -t eagle-eye-ai:1.0
