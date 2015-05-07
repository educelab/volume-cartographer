cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


set(SOURCES main.cpp voxel.cpp region.cpp voxel.h region.h)
add_executable(vc_region ${SOURCES})

target_link_libraries(vc_region ${PCL_LIBRARIES})
target_link_libraries(vc_region ${OpenCV_LIBS})
target_link_libraries(vc_region ${VC_LIBS} ${VC_REQS})
