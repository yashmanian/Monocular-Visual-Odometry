cmake_minimum_required(VERSION 3.5.1)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(MonocularVO)
find_package( OpenCV REQUIRED )
include_directories(include)
add_executable(vo src/main.cc src/VisOdom.cc include/VisOdom.hpp)
target_link_libraries( vo ${OpenCV_LIBS} )
