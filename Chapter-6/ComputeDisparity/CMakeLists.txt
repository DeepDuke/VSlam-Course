cmake_minimum_required(VERSION 3.10)
project(optical_flow)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

add_executable(compute_disparity ${PROJECT_SOURCE_DIR}/src/compute_disparity.cpp)
target_link_libraries(compute_disparity ${OpenCV_LIBS})
