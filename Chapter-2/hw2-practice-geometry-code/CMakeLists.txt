cmake_minimum_required(VERSION 3.10.2)
project(practice-geometry)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_Flags "-std=c++11")

#include_directories("/usr/include/eigen3")
add_executable(practice-geometry ${PROJECT_SOURCE_DIR}/src/practice-geometry.cpp)
