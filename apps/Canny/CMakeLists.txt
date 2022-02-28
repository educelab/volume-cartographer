# Canny app
set(target vc_canny_segment)
set(srcs
    CannySegment.cpp
    CannyThread.cpp
    CannyViewerWindow.cpp
    SliceCannyViewerWidget.cpp
)
add_executable(${target} ${srcs})
set_target_properties(${target} PROPERTIES
    AUTOMOC on
)
target_link_libraries(${target}
    VC::app_support
    VC::core
    VC::gui_support
    ${VC_FS_LIB}
    Boost::program_options
    opencv_core
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
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
