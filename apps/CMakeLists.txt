############
# GUI Apps #
############
if(VC_BUILD_GUI)
add_subdirectory(Canny)

add_subdirectory(Projection)

add_subdirectory(VC)

## Volume Server ##
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
    Qt6::Network
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
    Qt6::Network
)
target_include_directories(vc_volume_client PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
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
add_executable(vc_render_from_ppm src/RenderFromPPM.cpp src/RenderTexturing.cpp)
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

## Project Mesh ##
add_executable(vc_project_mesh src/ProjectMesh.cpp)
target_link_libraries(vc_project_mesh
    VC::core
    VC::texturing
    Boost::program_options
    ITKIOTransformBase
    ITKIOTransformHDF5
    ITKIOTransformInsightLegacy
    ITKIOTransformMatlab
)

## VolPkg Explorer ##
add_executable(vc_volpkg_explorer src/VolPkgExplorer.cpp)
target_link_libraries(vc_volpkg_explorer
    VC::core
    VC::graph
    Boost::program_options
    ${VC_FS_LIB}
)

# Install targets
if(VC_INSTALL_APPS)
install(
    TARGETS
        vc_layers_from_ppm
        vc_mesher
        vc_packager
        vc_segment
        vc_render
        vc_render_from_ppm
        vc_volpkg_explorer
        ${extra_apps}
    RUNTIME DESTINATION bin
    COMPONENT Programs
)
endif()
