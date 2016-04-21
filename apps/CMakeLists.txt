cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# VC.app
add_subdirectory(VC)

# Texture UI
add_subdirectory(VC-Texture)

# Mapping Viewer
add_subdirectory(mappingViewer)

# Mesher
add_executable(vc_mesher mesher.cpp)
target_link_libraries(vc_mesher ${VC_LIBS} ${VC_REQ_LIBS})

# Segment
add_executable(vc_segment segment.cpp)
target_link_libraries(vc_segment ${VC_LIBS} ${VC_REQ_LIBS})

# Texture
add_executable(vc_render render.cpp)
target_link_libraries(vc_render ${VC_LIBS} ${VC_REQ_LIBS})

# Metadata Editor
add_executable(vc_metaedit metadata.cpp)
target_link_libraries(vc_metaedit ${VC_LIBS} ${VC_REQ_LIBS})

# Slice Projection
add_executable(vc_projection projection.cpp)
target_link_libraries(vc_projection ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_projection2 projection2.cpp)
target_link_libraries(vc_projection2 ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

# Cloth Flattening Test
if(BULLET_FOUND)
    add_executable(vc_cloth cloth.cpp)
    target_link_libraries(vc_cloth ${VC_LIBS} vc_meshing ${VC_REQ_LIBS} ${ITK_LIBRARIES} ${VTK_LIBRARIES} ${OPENGL_LIBRARIES})
endif(BULLET_FOUND)

# Install targets
install(TARGETS vc_mesher vc_segment vc_metaedit vc_projection vc_render
        RUNTIME DESTINATION bin COMPONENT Programs)
