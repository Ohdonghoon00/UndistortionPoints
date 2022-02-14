cmake_minimum_required(VERSION 3.5.0)
project(UndistortionPoints)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

include_directories(
  include
  "/usr/include/eigen3" 
)

add_executable(UndirtortionPoints UndirtortionPoints.cpp)
target_link_libraries(UndirtortionPoints glog)



