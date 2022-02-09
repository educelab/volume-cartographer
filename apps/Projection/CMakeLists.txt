# Projection app
set(target vc_projection)
set(srcs
    Projection.cpp
    ProjectionViewer.cpp
)
add_executable(${target} MACOSX_BUNDLE WIN32 ${srcs})
set_target_properties(${target} PROPERTIES
    AUTOMOC on
)
target_link_libraries(${target}
    Boost::program_options
    opencv_core
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    VC::app_support
    VC::core
    VC::meshing
    ${VC_FS_LIB}
    vtkCommonCore
    vtkCommonDataModel
    vtkFiltersGeneral
)
target_include_directories(${target}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_BINARY_DIR}
)

if (VC_INSTALL_APPS)
    install(TARGETS ${target}
        BUNDLE DESTINATION . COMPONENT Programs
        RUNTIME DESTINATION bin COMPONENT Programs
    )
endif()
