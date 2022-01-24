############
# GUI Apps #
############
if(APPLE OR WIN32)
    find_package(DeployQt5 REQUIRED)
endif()

add_subdirectory(Canny)

add_subdirectory(VC)

if (VC_USE_ACVD OR VC_USE_OPTIONAL)
    add_subdirectory(VC-Texture)
endif()

#################
# CMD Line Apps #
#################

## Mesher ##
add_executable(vc_mesher src/Mesher.cpp)
target_link_libraries(vc_mesher
    VC::core
    VC::meshing
    ${VC_FS_LIB}
    Boost::program_options
)

## Packager ##
add_executable(vc_packager src/Packager.cpp src/SliceImage.cpp)
target_link_libraries(vc_packager
    VC::core
    VC::app_support
    ${VC_FS_LIB}
    Boost::program_options
)
target_include_directories(vc_packager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

## Segment ##
add_executable(vc_segment src/Segment.cpp)
target_link_libraries(vc_segment
    VC::app_support
    VC::core
    VC::segmentation
    VC::meshing
    Boost::program_options
)

## Rendering ##
add_executable(vc_render src/Render.cpp)
target_link_libraries(vc_render
    VC::app_support
    VC::core
    VC::graph
    ${VC_FS_LIB}
    Boost::program_options
)

## Render from PPM
add_executable(vc_render_from_ppm src/RenderFromPPM.cpp src/RenderIO.cpp src/RenderTexturing.cpp)
target_link_libraries(vc_render_from_ppm
    VC::core
    VC::texturing
    VC::meshing
    VC::app_support
    ${VC_FS_LIB}
    Boost::program_options
)
target_include_directories(vc_render_from_ppm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

## Layers ##
add_executable(vc_layers src/Layers.cpp)
target_link_libraries(vc_layers
    VC::app_support
    VC::core
    VC::texturing
    VC::meshing
    ${VC_FS_LIB}
    Boost::program_options
)

## PPM Generator
add_executable(vc_generate_ppm src/GeneratePPM.cpp)
target_link_libraries(vc_generate_ppm
    VC::app_support
    VC::core
    VC::meshing
    VC::texturing
    ${VC_FS_LIB}
)

## Layers from PPM
add_executable(vc_layers_from_ppm src/LayerFromPPM.cpp)
target_link_libraries(vc_layers_from_ppm
    VC::app_support
    VC::core
    VC::texturing
    ${VC_FS_LIB}
    Boost::program_options
)

## Metadata Editor ##
add_executable(vc_metaedit src/Metadata.cpp)
target_link_libraries(vc_metaedit
    VC::core
    Boost::program_options
    ${VC_FS_LIB}
)

## Slice Projection ##
add_executable(vc_projection src/Projection.cpp)
target_link_libraries(vc_projection
    VC::app_support
    VC::core
    VC::meshing
    ${VC_FS_LIB}
    Boost::program_options
    vtkCommonCore
    vtkCommonDataModel
    vtkFiltersGeneral
)

## VolPkg Explorer ##
add_executable(vc_volpkg_explorer src/VolPkgExplorer.cpp)
target_link_libraries(vc_volpkg_explorer
    VC::core
    VC::graph
    Boost::program_options
    ${VC_FS_LIB}
)

## Volume Server ##
if(Qt5_VERSION VERSION_GREATER_EQUAL 5.15)
add_executable(vc_volume_server
    src/VolumeServerApp.cpp
    src/VolumeServer.cpp
    include/vc/apps/server/VolumeServer.hpp
    include/vc/apps/server/VolumeProtocol.hpp)
set_target_properties(vc_volume_server PROPERTIES
    AUTOMOC on
)
target_link_libraries(vc_volume_server
    VC::app_support
    VC::core
    Boost::program_options
    ${VC_FS_LIB}
    Qt5::Network
)
target_include_directories(vc_volume_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

## Volume Client ##
add_executable(vc_volume_client
    src/VolumeClientApp.cpp
    src/VolumeClient.cpp
    include/vc/apps/server/VolumeClient.hpp
    include/vc/apps/server/VolumeProtocol.hpp)
set_target_properties(vc_volume_client PROPERTIES
    AUTOMOC on
)
target_link_libraries(vc_volume_client
    VC::app_support
    VC::core
    Boost::program_options
    ${VC_FS_LIB}
    Qt5::Network
)
target_include_directories(vc_volume_client PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
set(extra_apps vc_volume_server)
endif()

## Cloth Flattening ##
if (VC_BUILD_EXPERIMENTAL)
    add_executable(vc_cloth src/Cloth.cpp)
    target_link_libraries(vc_cloth
        VC::core
        VC::experimental
        VC::meshing
        VC::texturing
        Boost::program_options
        vtkIOPLY
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
        vc_render_from_ppm
        vc_projection
        vc_volpkg_explorer
        ${extra_apps}
    RUNTIME DESTINATION bin
    COMPONENT Programs
)
endif()
