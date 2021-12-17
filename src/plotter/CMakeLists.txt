# set minimum cmake version
cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# project name and language
project(plotter)

#create Plotter Library
add_library(plotter STATIC eggplot.cpp linespec.cpp)