cmake_minimum_required(VERSION 3.10)
project(optical_flow)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

add_executable(direct_method ${PROJECT_SOURCE_DIR}/src/direct_method.cpp)
target_link_libraries(direct_method ${OpenCV_LIBS} Sophus::Sophus ${Pangolin_LIBRARIES})
