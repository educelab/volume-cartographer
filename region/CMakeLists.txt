cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(REGION_SOURCES main.cpp voxel.cpp region.cpp voxel.h region.h)
add_executable(vc_region ${REGION_SOURCES})

target_link_libraries(vc_region ${VC_LIBS} ${VC_REQ_LIBS})
