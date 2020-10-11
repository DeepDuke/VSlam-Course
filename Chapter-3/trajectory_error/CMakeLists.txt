cmake_minimum_required(VERSION 3.10)
project(trajectory_error)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

#include_directories(${Sophus_INCLUDE_DIRS})

add_executable(trajectory_error ${PROJECT_SOURCE_DIR}/src/trajectory_error.cpp)
target_link_libraries(trajectory_error Sophus::Sophus ${Pangolin_LIBRARIES})
