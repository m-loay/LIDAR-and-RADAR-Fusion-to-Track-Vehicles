# set minimum cmake version
cmake_minimum_required(VERSION 2.8.9 FATAL_ERROR)

# project name and language
project(plotter)

#set standard of C++11 and build type to debug
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Debug)

#create Plotter Library
add_library(plotter STATIC eggplot.cpp linespec.cpp)