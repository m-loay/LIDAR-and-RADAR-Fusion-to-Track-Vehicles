# set minimum cmake version
cmake_minimum_required(VERSION 2.8.9 FATAL_ERROR)

# project name and language
project(kflib)

#set standard of C++11 and build type to debug
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Debug)

#create kalman filter library Library
add_library(kflib STATIC kf_predict.cpp kf_update.cpp kf_ut.cpp) 