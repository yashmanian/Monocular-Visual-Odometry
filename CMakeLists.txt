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

file(GLOB PROJECT_INC "include/*.h"
                      "include/*.hpp")
file(GLOB PROJECT_SRC "src/srcFiles/*.cpp")

add_executable(vo src/Tests/epiLines.cpp ${PROJECT_SRC})
add_executable(opTest src/Tests/OpTest.cpp ${PROJECT_SRC})


target_link_libraries( vo ${OpenCV_LIBS} Eigen3::Eigen)
target_link_libraries( opTest ${OpenCV_LIBS} Eigen3::Eigen)