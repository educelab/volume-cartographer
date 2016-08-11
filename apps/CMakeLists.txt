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
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
)
target_include_directories(vc_mesher
    PUBLIC
        ${PCL_COMMON_INCLUDE_DIRS}
        ${PCL_IO_INCLUDE_DIRS}
)

## Packager ##
add_executable(vc_packager src/packager.cpp src/SliceImage.cpp)
target_link_libraries(vc_packager
    vc_volumepkg
    vc_common
    Boost::filesystem
    Boost::program_options
)
# Special - need to include SliceImage.h
target_include_directories(vc_packager PUBLIC include)

## Segment ##
add_executable(vc_segment src/segment.cpp)
target_link_libraries(vc_segment
    vc_volumepkg
    vc_segmentation
    Boost::program_options
    ${PCL_IO_LIBRARIES}
)
target_include_directories(vc_segment PUBLIC ${PCL_IO_INCLUDE_DIRS})

## Rendering ##
add_executable(vc_render src/render.cpp)
target_link_libraries(vc_render
    vc_common
    vc_volumepkg
    vc_texturing
    vc_meshing
    Boost::filesystem
    Boost::program_options
)

## Metadata Editor ##
add_executable(vc_metaedit src/metadata.cpp)
target_link_libraries(vc_metaedit vc_volumepkg)

## Slice Projection ##
# Old
add_executable(vc_projection src/projection.cpp)
target_link_libraries(vc_projection
    vc_common
    vc_volumepkg
)

# New
add_executable(vc_projection2 src/projection2.cpp)
target_link_libraries(vc_projection2
    vc_volumepkg
    vc_common
    vtkCommonCore
    vtkCommonDataModel
    vtkIOPLY
)

## Cloth Flattening ##
if (VC_USE_BULLET)
    add_executable(vc_cloth src/cloth.cpp)
    target_link_libraries(vc_cloth
        vc_common
        vc_meshing
        vc_texturing
        Boost::program_options
        ${BULLET_LIBRARIES}
    )
endif()

# Install targets
install(TARGETS vc_mesher vc_packager vc_segment vc_render vc_metaedit vc_projection vc_projection2
        RUNTIME DESTINATION bin COMPONENT Programs)
