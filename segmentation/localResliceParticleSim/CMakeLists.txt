cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_library(localResliceParticleSim OBJECT localResliceParticleSim.cpp fittedcurve.cpp intensitymap.cpp)
