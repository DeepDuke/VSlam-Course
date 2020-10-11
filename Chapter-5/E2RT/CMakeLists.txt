project(E2RT-practice)
cmake_minimum_required(VERSION 3.10)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

find_package(Sophus REQUIRED)

add_executable(E2RT-practice ${PROJECT_SOURCE_DIR}/src/E2Rt.cpp)
target_link_libraries(E2RT-practice Sophus::Sophus)
