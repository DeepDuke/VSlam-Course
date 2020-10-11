cmake_minimum_required(VERSION 3.10)
project(gaussian_newton)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

add_executable(gaussian_newton ${PROJECT_SOURCE_DIR}/src/gaussnewton.cpp)
target_link_libraries(gaussian_newton ${OpenCV_LIBS})

