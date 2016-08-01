project(libvc_texturing VERSION 0.1 LANGUAGES CXX)

set(VC_TEXTURING_SRCS
    src/compositeTexture.cpp
    src/compositeTextureV2.cpp
    src/layering.cpp
    src/simpleUV.cpp
)

add_library(vc_texturing ${VC_TEXTURING_SRCS})

target_include_directories(vc_texturing
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(vc_texturing
    PUBLIC
        vc_common
        vc_volumepkg
    PRIVATE
        vc_meshing
)
