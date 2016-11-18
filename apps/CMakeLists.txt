############
# GUI Apps #
############
add_subdirectory(VC)

if (VC_USE_ACVD)
    add_subdirectory(VC-Texture)
endif()

#################
# CMD Line Apps #
#################

## Mesher ##
add_executable(vc_mesher src/mesher.cpp)
target_link_libraries(vc_mesher
    vc_meshing
)

## Packager ##
add_executable(vc_packager src/packager.cpp src/SliceImage.cpp)
target_link_libraries(vc_packager
    vc_core
    Boost::filesystem
    Boost::program_options
)
# Special - need to include SliceImage.h
target_include_directories(vc_packager PUBLIC include)

## Segment ##
add_executable(vc_segment src/segment.cpp)
target_link_libraries(vc_segment
    vc_core
    vc_segmentation
    vc_meshing
    Boost::program_options
)

## Rendering ##
add_executable(vc_render src/render.cpp)
target_link_libraries(vc_render
    vc_core
    vc_texturing
    vc_meshing
    Boost::filesystem
    Boost::program_options
)

## Metadata Editor ##
add_executable(vc_metaedit src/metadata.cpp)
target_link_libraries(vc_metaedit vc_core)

## Slice Projection ##
# Old
add_executable(vc_projection src/projection.cpp)
target_link_libraries(vc_projection
    vc_core
)

# New
add_executable(vc_projection2 src/projection2.cpp)
target_link_libraries(vc_projection2
    vc_core
    vtkCommonCore
    vtkCommonDataModel
    vtkFiltersGeneral
    vtkIOPLY
)

## Cloth Flattening ##
if (VC_USE_BULLET)
    add_executable(vc_cloth src/cloth.cpp)
    target_link_libraries(vc_cloth
        vc_core
        vc_meshing
        vc_texturing
        Boost::program_options
        ${BULLET_LIBRARIES}
    )
endif()

# Install targets
install(
    TARGETS
        vc_mesher
        vc_packager
        vc_segment
        vc_render
        vc_metaedit
        vc_projection
        vc_projection2
    RUNTIME DESTINATION bin
    COMPONENT Programs
)
