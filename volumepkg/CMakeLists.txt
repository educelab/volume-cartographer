cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_CXX_FLAGS "-std=c++11")

set(VOLUMEPKG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(VOLUMEPKG_INCLUDE_DIR ${VOLUMEPKG_INCLUDE_DIR} PARENT_SCOPE)

if(TARGET volumepkg)
message("volumepkg is already defined")
else()
include_directories(${VOLUMEPKG_INCLUDE_DIR})

add_library(volumepkg STATIC volumepkgcfg.cpp volumepkg.cpp)

endif()