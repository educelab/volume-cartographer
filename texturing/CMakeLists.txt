project(libvc_texturing VERSION 0.1 LANGUAGES CXX)

set(VC_TEXTURING_SRCS
    src/compositeTexture.cpp
    src/compositeTextureV2.cpp
    src/simpleUV.cpp
    src/AngleBasedFlattening.cpp
)
set(VC_TEXTURING_DEPENDENCIES_PUBLIC
    vc_common
    vc_volumepkg
)
set(VC_TEXTURING_DEPENDENCIES_PRIVATE
    vc_meshing
    vc_external
)
set(VC_TEXTURING_DEFINITIONS "")

if (VC_USE_LIBIGL)
    list(APPEND VC_TEXTURING_SRCS src/LeastSquaresConformalMapping.cpp)
    list(APPEND VC_TEXTURING_DEPENDENCIES_PRIVATE libigl)
    list(APPEND VC_TEXTURING_DEFINITIONS USE_IGL)
endif()
if (VC_USE_BULLET)
    list(APPEND VC_TEXTURING_SRC src/ClothModelingUVMapping.cpp)
    list(APPEND VC_TEXTURING_DEPENDENCIES_PUBLIC bullet)
    list(APPEND VC_TEXTURING_DEFINITIONS USE_BULLET)
endif()

add_library(vc_texturing ${VC_TEXTURING_SRCS})

target_compile_definitions(vc_texturing
    PRIVATE
        ${VC_TEXTURING_DEFINITIONS}
)

target_include_directories(vc_texturing
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(vc_texturing
    PUBLIC
        ${VC_TEXTURING_DEPENDENCIES_PUBLIC}
    PRIVATE
        ${VC_TEXTURING_DEPENDENCIES_PRIVATE}
)

install(TARGETS vc_texturing
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX})

export(TARGETS vc_texturing FILE VCTexturing.cmake)
