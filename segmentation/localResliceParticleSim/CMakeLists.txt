cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_compile_options(-fdiagnostics-color=always)
add_compile_options(-Wno-missing-braces)
add_compile_options(-Wno-unused-local-typedef)
add_compile_options(-Wno-c++11-narrowing)
add_library(localResliceParticleSim OBJECT localResliceParticleSim.cpp fittedcurve.cpp intensitymap.cpp)
