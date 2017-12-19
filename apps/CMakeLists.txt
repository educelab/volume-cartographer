############
# GUI Apps #
############
add_subdirectory(VC)

if (VC_USE_ACVD OR VC_USE_ALL)
    add_subdirectory(VC-Texture)
endif()

#################
# CMD Line Apps #
#################

## Mesher ##
add_executable(vc_mesher src/Mesher.cpp)
target_link_libraries(vc_mesher
    VC::meshing
)

## Packager ##
add_executable(vc_packager src/Packager.cpp src/SliceImage.cpp)
target_link_libraries(vc_packager
    VC::core
    Boost::filesystem
    Boost::program_options
)
# Special - need to include SliceImage.h
target_include_directories(vc_packager PUBLIC include)

## Segment ##
add_executable(vc_segment src/Segment.cpp)
target_link_libraries(vc_segment
    VC::core
    VC::segmentation
    VC::meshing
    Boost::program_options
)

## Rendering ##
add_executable(vc_render src/Render.cpp)
target_link_libraries(vc_render
    VC::core
    VC::texturing
    VC::meshing
    Boost::filesystem
    Boost::program_options
)

## Layers ##
add_executable(vc_layers src/Layers.cpp)
target_link_libraries(vc_layers
    VC::core
    VC::texturing
    VC::meshing
    Boost::filesystem
    Boost::program_options
)

add_executable(vc_generate_ppm src/GeneratePPM.cpp)
target_link_libraries(vc_generate_ppm
    VC::core
    VC::meshing
    VC::texturing
    Boost::filesystem
)

add_executable(vc_layers_from_ppm src/LayerFromPPM.cpp)
target_link_libraries(vc_layers_from_ppm
    VC::core
    VC::external
    VC::texturing
    Boost::filesystem
)

## Metadata Editor ##
add_executable(vc_metaedit src/Metadata.cpp)
target_link_libraries(vc_metaedit
    VC::core
    Boost::program_options
    Boost::filesystem
)

## Slice Projection ##
add_executable(vc_projection src/Projection.cpp)
target_link_libraries(vc_projection
    VC::core
    VC::meshing
    Boost::filesystem
    Boost::program_options
    vtkCommonCore
    vtkCommonDataModel
    vtkFiltersGeneral
)

add_executable(vc_volpkg_explorer src/VolPkgExplorer.cpp)
target_link_libraries(vc_volpkg_explorer
    VC::core
    Boost::program_options
    Boost::filesystem
)

## Cloth Flattening ##
if (VC_USE_BULLET OR VC_USE_ALL)
    add_executable(vc_cloth src/Cloth.cpp)
    target_link_libraries(vc_cloth
        VC::core
        VC::meshing
        VC::texturing
        Boost::program_options
        ${BULLET_LIBRARIES}
    )
endif()

# Install targets
if(VC_INSTALL_APPS)
install(
    TARGETS
        vc_layers
        vc_layers_from_ppm
        vc_mesher
        vc_packager
        vc_segment
        vc_render
        vc_projection
        vc_volpkg_explorer
    RUNTIME DESTINATION bin
    COMPONENT Programs
)
endif()
