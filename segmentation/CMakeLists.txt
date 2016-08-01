project(libvc_segmentation VERSION 0.1 LANGUAGES CXX)

set(VC_SEGMENTATION_SRCS
    src/chain.cpp
    src/common.cpp
    src/energymetrics.cpp
    src/fittedcurve.cpp
    src/intensitymap.cpp
    src/localResliceParticleSim.cpp
    src/particle.cpp
    src/structureTensorParticleSim.cpp
)

set(VC_SEGMENTATION_DEPENDENCIES
    vc_common
    vc_volumepkg
)

add_library(vc_segmentation ${VC_SEGMENTATION_SRCS})

target_include_directories(vc_segmentation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_options(vc_segmentation
    PRIVATE
        -Wno-c++11-narrowing
)

target_link_libraries(vc_segmentation
    PUBLIC
        vc_common
        vc_volumepkg
)

install(TARGETS vc_segmentation
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX})

export(TARGETS vc_segmentation FILE VCSegmentation.cmake)
