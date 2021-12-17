# set minimum cmake version
cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# project name and language
project(tools)

include_directories ("${CMAKE_SOURCE_DIR}")
add_library(tools STATIC tools.cpp)
