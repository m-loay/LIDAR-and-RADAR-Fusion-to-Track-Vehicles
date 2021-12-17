# set minimum cmake version
cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# project name and language
project(highway)

include_directories ("${CMAKE_SOURCE_DIR}")
add_library(highway STATIC ukf.cpp highway.cpp render/render.cpp)
