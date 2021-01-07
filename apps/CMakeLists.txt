############
# GUI Apps #
############
if(APPLE OR WIN32)
    find_package(DeployQt5 REQUIRED)
endif()

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
    Boost::filesystem
    Boost::program_options
)

## Packager ##
add_executable(vc_packager src/Packager.cpp src/SliceImage.cpp)
target_link_libraries(vc_packager
    VC::core
    VC::app_support
    Boost::filesystem
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
add_executable(vc_render
    src/Render.cpp
    src/RenderIO.cpp
    src/RenderMeshing.cpp
    src/RenderTexturing.cpp
)
target_link_libraries(vc_render
    VC::app_support
    VC::core
    VC::texturing
    VC::meshing
    Boost::filesystem
    Boost::program_options
    vtkFiltersCore
)
target_include_directories(vc_render PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

## Render from PPM
add_executable(vc_render_from_ppm src/RenderFromPPM.cpp src/RenderIO.cpp src/RenderTexturing.cpp)
target_link_libraries(vc_render_from_ppm
    VC::core
    VC::texturing
    VC::meshing
    VC::app_support
    Boost::filesystem
    Boost::program_options
)
target_include_directories(vc_render_from_ppm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
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

## PPM Generator
add_executable(vc_generate_ppm src/GeneratePPM.cpp)
target_link_libraries(vc_generate_ppm
    VC::core
    VC::meshing
    VC::texturing
    Boost::filesystem
)

## Layers from PPM
add_executable(vc_layers_from_ppm src/LayerFromPPM.cpp)
target_link_libraries(vc_layers_from_ppm
    VC::core
    VC::external
    VC::texturing
    Boost::filesystem
    Boost::program_options
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
    VC::app_support
    VC::core
    VC::meshing
    Boost::filesystem
    Boost::program_options
    vtkCommonCore
    vtkCommonDataModel
    vtkFiltersGeneral
)

## VolPkg Explorer ##
add_executable(vc_volpkg_explorer src/VolPkgExplorer.cpp)
target_link_libraries(vc_volpkg_explorer
    VC::core
    Boost::program_options
    Boost::filesystem
)

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
    VC::core
    Boost::program_options
    Boost::filesystem
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
    VC::core
    Boost::program_options
    Boost::filesystem
    Qt5::Network
)
target_include_directories(vc_volume_client PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

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
        vc_volume_server
    RUNTIME DESTINATION bin
    COMPONENT Programs
)
endif()
