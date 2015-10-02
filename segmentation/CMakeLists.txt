cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(SEGMENTATION_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(SEGMENTATION_INCLUDE_DIR ${SEGMENTATION_INCLUDE_DIR} PARENT_SCOPE)

if(TARGET segmentation)
  message("segmentation is already defined")
else()
  add_subdirectory(structureTensorParticleSim)
  add_subdirectory(localResliceParticleSim)
  add_library(vc_segmentation STATIC $<TARGET_OBJECTS:structureTensorParticleSim> $<TARGET_OBJECTS:localResliceParticleSim> ../volumepkg/Slice.cpp ../volumepkg/Slice.h)
endif()