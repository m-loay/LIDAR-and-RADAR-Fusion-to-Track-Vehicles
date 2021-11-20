# set minimum cmake version
cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# project name and language
project(tools)

#set standard of C++11 and build type to debug
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories ("${CMAKE_SOURCE_DIR}")
add_library(tools STATIC tools.cpp)
