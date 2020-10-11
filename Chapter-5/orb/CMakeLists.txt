project(orb-practice)
cmake_minimum_required(VERSION 3.10)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

add_executable(orb-practice ${PROJECT_SOURCE_DIR}/src/computeORB.cpp)
target_link_libraries(orb-practice ${OpenCV_LIBS})

