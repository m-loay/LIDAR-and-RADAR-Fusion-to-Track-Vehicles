cmake_minimum_required(VERSION 3.12)
project(testCases)

#create testCases Library
include_directories ("${KFLIB_INCLIDE_DIR}")
include_directories ("${KFAPP_INCLIDE_DIR}")
include_directories ("${TOOLS_INCLIDE_DIR}")

add_library(testCases STATIC test_cases.cpp)


