cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_FLAGS "-Wall -g -std=c++11")

project(Debugger3D)

include_directories(src)
add_subdirectory(src)
