# CMake change for tutorial
cmake_minimum_required(VERSION 3.5.1)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

project(MonocularVO)
find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED NO_MODULE )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories(include)

file(MAKE_DIRECTORY "bin")

add_executable(vo src/Tests/epiLines.cpp src/srcFiles/FeatTrack.cpp src/srcFiles/epipolar.cpp src/srcFiles/PointOps.cpp include/FeatTrack.hpp include/epipolar.hpp include/PointOps.hpp)

target_link_libraries( vo ${OpenCV_LIBS} Eigen3::Eigen)
