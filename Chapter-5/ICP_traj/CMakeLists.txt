project(ICP_traj)
cmake_minimum_required(VERSION 3.10)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

add_executable(ICP_traj ${PROJECT_SOURCE_DIR}/src/ICP_traj.cpp)
target_link_libraries(ICP_traj Sophus::Sophus ${Pangolin_LIBRARIES})
