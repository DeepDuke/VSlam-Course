cmake_minimum_required(VERSION 3.10)
project(optical_flow)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

add_executable(optical_flow ${PROJECT_SOURCE_DIR}/src/optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS})
