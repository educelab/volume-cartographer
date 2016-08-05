project(libvc_volumepkg VERSION 0.1 LANGUAGES CXX)
set(target vc_volumepkg)

set(srcs
    src/volumepkg.cpp
)

add_library(${target} ${srcs})
target_include_directories(vc_volumepkg
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${target}
    PUBLIC
        vc_common
        vc_external
    PRIVATE
        vc_meshing
)

install(TARGETS ${target}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX})
export(TARGETS ${target} FILE VCVolumepkg.cmake)
