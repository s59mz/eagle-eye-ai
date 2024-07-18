#
# Copyright 2019 Xilinx Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

cmake_minimum_required(VERSION 3.5)

project(eagle-eye)

SET(CMAKE_INSTALL_PREFIX $ENV{OECORE_TARGET_SYSROOT})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing -fpermissive -rdynamic -Wl,--no-undefined" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++14")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(GStreamer REQUIRED)
find_package(OpenCV REQUIRED)

SET(INSTALL_PATH "opt/xilinx/kr260-eagle-eye")

add_library(vvas_xpp SHARED src/vvas_xpp_pipeline.c)
target_include_directories(vvas_xpp PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(vvas_xpp
  jansson vvasutil-2.0 gstvvasinfermeta-2.0)
install(TARGETS vvas_xpp DESTINATION ${INSTALL_PATH}/lib)

add_library(vvas_airender SHARED src/vvas_airender.cpp)
target_include_directories(vvas_airender PRIVATE ${GSTREAMER_INCLUDE_DIRS})
target_link_libraries(vvas_airender 
    jansson vvasutil-2.0 gstvvasinfermeta-2.0 
    gstreamer-1.0 gstbase-1.0 glib-2.0
    opencv_core opencv_video opencv_videoio opencv_imgproc)
install(TARGETS vvas_airender DESTINATION ${INSTALL_PATH}/lib)


install(FILES
    README
    DESTINATION ${INSTALL_PATH}/
    RENAME      README_EAGLE_AI
    )

install(DIRECTORY
    config/facedetect
    DESTINATION ${INSTALL_PATH}/share/vvas/)

install(DIRECTORY
    models/kr260/densebox_640_360
    DESTINATION ${INSTALL_PATH}/share/vitis_ai_library/models/)

set(VERSION "1.0.1")
set(CPACK_PACKAGE_VERSION ${VERSION})
set(CPACK_GENERATOR "RPM")
set(CPACK_PACKAGE_NAME "${CMAKE_PROJECT_NAME}")
set(CPACK_PACKAGE_RELEASE 1)
set(CPACK_PACKAGE_CONTACT "Yuxiz@xilinx.com")
set(CPACK_PACKAGE_VENDOR "s59mz")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
set(CPACK_RPM_PACKAGE_ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})
set(CPACK_PACKAGING_INSTALL_PREFIX "/")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_PACKAGE_RELEASE}.${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_RPM_SPEC_MORE_DEFINE "%define _build_id_links none")
include(CPack)

