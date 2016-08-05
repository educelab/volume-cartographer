project(libvc_volumepkg VERSION 0.1 LANGUAGES CXX)

set(VC_VOLUMEPKG_SRCS
    src/volumepkg.cpp
)

add_library(vc_volumepkg ${VC_VOLUMEPKG_SRCS})

target_include_directories(vc_volumepkg
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(vc_volumepkg
    PUBLIC
        vc_common
        vc_external
    PRIVATE
        vc_meshing
)

install(TARGETS vc_volumepkg
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX})

export(TARGETS vc_volumepkg FILE VCVolumepkg.cmake)
