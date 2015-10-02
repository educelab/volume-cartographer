cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(VOLUMEPKG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(VOLUMEPKG_INCLUDE_DIR ${VOLUMEPKG_INCLUDE_DIR} PARENT_SCOPE)

if(TARGET volumepkg)
message("volumepkg is already defined")
else()
add_library(vc_volumepkg STATIC volumepkgcfg.cpp volumepkg.cpp ../texture/CPlyHelper.cpp ../texture/CMesh.cpp Slice.cpp Slice.h)
#install(TARGETS vc_volumepkg DESTINATION lib/vc)
#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} DESTINATION include/vc FILES_MATCHING PATTERN "*.h")
endif()
