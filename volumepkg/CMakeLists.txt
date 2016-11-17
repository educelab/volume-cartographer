project(libvc_volumepkg VERSION 0.1 LANGUAGES CXX)
set(target vc_volumepkg)

set(srcs
    src/volumepkg.cpp
)

add_library(${target} ${srcs})
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${target}
    PUBLIC
        vc_common
        vc_external
)

install(TARGETS ${target}
    ARCHIVE DESTINATION lib COMPONENT Libraries
    LIBRARY DESTINATION lib COMPONENT Libraries
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX} COMPONENT Includes)
export(TARGETS ${target} FILE VCVolumepkg.cmake)
