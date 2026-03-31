cmake_minimum_required(VERSION 3.10)
project(line_detection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED)

add_executable(line_detection main.cpp)

target_link_libraries(line_detection ${OpenCV_LIBS})
