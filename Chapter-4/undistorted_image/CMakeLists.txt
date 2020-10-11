cmake_minimum_required(VERSION 3.10)
project(undistorted_image)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

add_executable(undistort_image ${PROJECT_SOURCE_DIR}/src/undistort_image.cpp)
target_link_libraries(undistort_image ${OpenCV_LIBS})

