cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

add_executable(vc_texture main.cpp CMesh.cpp CPlyHelper.cpp modules/meshTexturing.cpp)
target_link_libraries(vc_texture ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_meshnorm mesh_normalize.cpp CMesh.cpp CPlyHelper.cpp)
target_link_libraries(vc_meshnorm ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_sliceProjection sliceProjection.cpp CMesh.cpp CPlyHelper.cpp sliceIntersection.cpp)
target_link_libraries(vc_sliceProjection ${VC_LIBS} ${VC_REQ_LIBS})

# Install targets
install(TARGETS vc_texture
        RUNTIME DESTINATION bin COMPONENT Programs)

install(TARGETS vc_meshnorm vc_sliceProjection
        RUNTIME DESTINATION bin COMPONENT Utilities)
