cmake_minimum_required(VERSION 3.10)
project(draw_trajectory)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(${Sophus_INCLUDE_DIRS})

add_executable(draw_trajectory ${PROJECT_SOURCE_DIR}/src/draw_trajectory.cpp)
target_link_libraries(draw_trajectory Sophus::Sophus ${Pangolin_LIBRARIES})
