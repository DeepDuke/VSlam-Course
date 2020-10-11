project(GN-BA-practice)
cmake_minimum_required(VERSION 3.10)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

find_package(Sophus REQUIRED)

add_executable(GN-BA-practice ${PROJECT_SOURCE_DIR}/src/GN-BA.cpp)
target_link_libraries(GN-BA-practice Sophus::Sophus)
