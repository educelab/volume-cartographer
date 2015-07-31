cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_executable(vc_packager slice.h slice.cpp packager.h packager.cpp)
target_link_libraries(vc_packager ${VC_LIBS} ${VC_REQ_LIBS})

# Install targets
install(TARGETS vc_packager
        RUNTIME DESTINATION bin COMPONENT Programs)
