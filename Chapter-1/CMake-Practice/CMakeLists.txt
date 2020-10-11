project(CmakePractice)
cmake_minimum_required(VERSION 3.10)

set(CMAKE_BUILD_TYPE "Release")

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(hello SHARED ${PROJECT_SOURCE_DIR}/src/hello.cpp)
add_executable(sayhello ${PROJECT_SOURCE_DIR}/src/useHello.cpp)
target_link_libraries(sayhello hello)

install(FILES ${PROJECT_SOURCE_DIR}/include/hello.h DESTINATION /usr/local/include)
install(TARGETS hello
	LIBRARY DESTINATION /usr/local/lib
	)

