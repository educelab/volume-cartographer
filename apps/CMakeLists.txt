cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# VC.app
add_subdirectory(VC)

# Texture UI
add_subdirectory(VC-Texture)

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

# Cloth Flattening Test
add_executable(vc_cloth cloth.cpp)
target_link_libraries(vc_cloth ${VC_LIBS} ${VC_REQ_LIBS} ${ITK_LIBRARIES} ${OPENGL_LIBRARIES})

# Install targets
install(TARGETS vc_mesher vc_segment vc_metaedit vc_projection vc_render
        RUNTIME DESTINATION bin COMPONENT Programs)
