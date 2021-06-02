cmake_minimum_required(VERSION 2.8)
project(arcar)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")

find_package(OpenCV 3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

#include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(vanishing_point_based_atti_cali vanishing_point_based_atti_cali.cc)
add_executable(process_main process_main.cc)
target_link_libraries(process_main vanishing_point_based_atti_cali ${OpenCV_LIBS})