# Projection app
set(target vc_projection)
set(srcs
    PPMProjectionThread.cpp
    PPMProjectionViewerWidget.cpp
    Projection.cpp
    ProjectionSettingsWidget.cpp
    SliceProjectionThread.cpp
    ProjectionViewerWindow.cpp
    SliceProjectionViewerWidget.cpp
)
add_executable(${target} ${srcs})
set_target_properties(${target} PROPERTIES
    AUTOMOC on
)
target_link_libraries(${target}
    Boost::program_options
    opencv_core
    opencv_imgproc
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    VC::app_support
    VC::gui_support
    VC::core
    VC::meshing
    ${VC_FS_LIB}
    VTK::CommonCore
    VTK::CommonDataModel
    VTK::FiltersGeneral
)
target_include_directories(${target}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_BINARY_DIR}
)

# NOTE: Linked against system Qt libs
if (VC_INSTALL_APPS)
    install(TARGETS ${target}
        BUNDLE DESTINATION . COMPONENT Programs
        RUNTIME DESTINATION bin COMPONENT Programs
    )
endif()
