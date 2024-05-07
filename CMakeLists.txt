cmake_minimum_required(VERSION 3.0.2)
project(TrafficLight)
add_definitions(-std=c++11)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../../bin)
set(CMAKE_CXX_FLAGS "-Werror=return-type -Wno-error=deprecated-declarations -Wno-deprecated-declarations -Werror")
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(ARCH_PLATFORM "arm64") 
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64") 
  if(TARGET_CPU STREQUAL "arm")
	  set (CMAKE_C_COMPILER "aarch64-linux-gnu-gcc") 
	  set (CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
	  set(ARCH_PLATFORM "arm64")
  else()
	  set(ARCH_PLATFORM "amd64")
  endif() 
endif()

if(${HISTOGRAM} STREQUAL on)
  add_definitions(-DHISTOGRAM)
endif()

include_directories(
  Alg
  Alg/log
  Alg/info
  Alg/TrafficLightClassify
  Alg/Bytetrack
  Alg/DistanceTrafficLight
  InterFace
  RosNode
  RosNode/common
  RosNode/common/log
  ../../
  ../../Common
  ../../Common/log
  ../../Common/Configure
  ../../Common/Error
  ../../Common/info
  ../../Common/Interface
  ../../Common/Report
  ../../message
  ../../../thirdparty/include
  ../../../thirdparty/include/eigen3
  ../../../thirdparty/include/ros
  ../../../thirdparty/include/yaml-cpp
  ../../../thirdparty/include/jsoncpp
  ../../../thirdparty/include/opencv
  ../../../thirdparty/include/nvidia/${ARCH_PLATFORM}/TensorRT
)
LINK_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}/../../bin/lib
  ${PROJECT_SOURCE_DIR}/../../../thirdparty/lib/${ARCH_PLATFORM}/yaml-cpp
  ${PROJECT_SOURCE_DIR}/../../../thirdparty/lib/${ARCH_PLATFORM}/jsoncpp
  ${PROJECT_SOURCE_DIR}/../../../thirdparty/lib/${ARCH_PLATFORM}/opencv
  ${PROJECT_SOURCE_DIR}/../../../thirdparty/lib/${ARCH_PLATFORM}/log4cplus
  ${PROJECT_SOURCE_DIR}/../../../thirdparty/lib/${ARCH_PLATFORM}/ros
  ${PROJECT_SOURCE_DIR}/../../../thirdparty/lib/${ARCH_PLATFORM}/nvidia/cuda
  ${PROJECT_SOURCE_DIR}/../../../thirdparty/lib/${ARCH_PLATFORM}/nvidia/cudnn
  ${PROJECT_SOURCE_DIR}/../../../thirdparty/lib/${ARCH_PLATFORM}/nvidia/TensorRT
  ${PROJECT_SOURCE_DIR}/../../../thirdparty/lib/${ARCH_PLATFORM}/boost
  ${PROJECT_SOURCE_DIR}/../../../thirdparty/lib/${ARCH_PLATFORM}/common
)
add_executable(TrafficLight 
  Alg/Bytetrack/BytekalmanFilter.cpp
  Alg/Bytetrack/BYTETracker.cpp
  Alg/Bytetrack/lapjv.cpp
  Alg/Bytetrack/STrack.cpp
  Alg/Bytetrack/utils.cpp
  Alg/Bytetrack/cmc.cpp
  Alg/TrafficLightClassify/NetEngine.cpp
  Alg/TrafficLightClassify/TrafficLight.cpp
  Alg/log/Base.cpp
  InterFace/TrafficLightInput.cpp
  InterFace/TrafficLightInterface.cpp
  InterFace/TrafficLightOutput.cpp
  RosNode/main.cpp
  RosNode/RosNode.cpp
)

target_link_libraries(TrafficLight
  opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_video  opencv_features2d
  Common
  yaml-cpp
  jsoncpp
  log4cplus
  roslib roscpp rosbag rostime rosbag_storage cpp_common roscpp_serialization rosconsole xmlrpcpp cv_bridge
  nvparsers nvonnxparser nvinfer
  cudnn
  cudart cusolver nvToolsExt cuda cublas cublasLt nvrtc
  boost_filesystem boost_system
  pthread
)
