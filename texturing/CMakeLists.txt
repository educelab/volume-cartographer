cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

add_executable(vc_texture2 main.cpp UPointMapping.cpp plyHelper.cpp)
target_link_libraries(vc_texture2 ${VC_LIBS} ${VC_REQ_LIBS} ${ITK_LIBRARIES})

#add_executable(vc_layering layering.cpp plyHelper.cpp meshUtils.cpp)
#target_link_libraries(vc_layering ${VC_LIBS} ${VC_REQ_LIBS} ${ITK_LIBRARIES})

# Install targets
install(TARGETS vc_texture2
        RUNTIME DESTINATION bin COMPONENT Demos)
