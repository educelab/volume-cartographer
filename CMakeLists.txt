cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_policy(SET CMP0048 NEW)
SET(CMAKE_CXX_FLAGS "-std=c++11")

project(VC VERSION 1.0)

add_subdirectory(volumepkg)

add_subdirectory(pathgen)
add_subdirectory(reslicer)
add_subdirectory(scalpel)
add_subdirectory(simulation)
add_subdirectory(texture)