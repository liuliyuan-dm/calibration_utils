cmake_minimum_required(VERSION 2.8)
project(intrinsics_calibration)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")
#list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(OpenCV 3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} "/usr/local/include/eigen3/")
include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(camera_intrinsics_calibration camera_intrinsics_calibration.cc)

add_executable(camera_calibration_main camera_calibration_main.cc)
target_link_libraries(camera_calibration_main ${OpenCV_LIBS} camera_intrinsics_calibration)

add_executable(camera_pyrdown_main camera_pyrdown_main.cc)
target_link_libraries(camera_pyrdown_main ${OpenCV_LIBS} camera_intrinsics_calibration)