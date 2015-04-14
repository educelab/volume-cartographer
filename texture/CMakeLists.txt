cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

add_executable(vc_texture main.cpp CMesh.cpp CPlyHelper.cpp modules/meshTexturing.cpp)

target_link_libraries(vc_texture ${PCL_LIBRARIES})
target_link_libraries(vc_texture ${OpenCV_LIBS})
target_link_libraries(vc_texture volumepkg)

add_executable(vc_meshnorm mesh_normalize.cpp CMesh.cpp CPlyHelper.cpp)

target_link_libraries(vc_meshnorm ${PCL_LIBRARIES})
target_link_libraries(vc_meshnorm ${OpenCV_LIBS})
target_link_libraries(vc_meshnorm volumepkg)

add_executable(vc_sliceProjection sliceProjection.cpp CMesh.cpp CPlyHelper.cpp sliceIntersection.cpp)

target_link_libraries(vc_sliceProjection ${PCL_LIBRARIES})
target_link_libraries(vc_sliceProjection ${OpenCV_LIBS})
target_link_libraries(vc_sliceProjection volumepkg)
