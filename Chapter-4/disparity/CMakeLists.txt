cmake_minimum_required(VERSION 3.10)
project(disparity)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

add_executable(disparity ${PROJECT_SOURCE_DIR}/src/disparity.cpp)
target_link_libraries(disparity ${OpenCV_LIBS} ${Pangolin_LIBRARIES})


