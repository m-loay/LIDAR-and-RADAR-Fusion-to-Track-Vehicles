# set minimum cmake version
cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# project name and language
project(ukfApp)

include_directories ("${KCONFIG_INCLIDE_DIR}")
include_directories ("${KFLIB_INCLIDE_DIR}")
add_library(ukfApp STATIC ukfApp.cpp)
