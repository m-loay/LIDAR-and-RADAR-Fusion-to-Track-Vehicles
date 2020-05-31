cmake_minimum_required(VERSION 2.8.9)
project(testCases)

#set standard of C++11 and build type to debug
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#create testCases Library
include_directories ("${KFLIB_INCLIDE_DIR}")
include_directories ("${KFAPP_INCLIDE_DIR}")
include_directories ("${TOOLS_INCLIDE_DIR}")

add_library(testCases STATIC test_cases.cpp)


