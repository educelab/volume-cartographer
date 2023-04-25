# Canny app
set(target vc_canny_segment)

set(srcs
    CannySegment.cpp
    CannyThread.cpp
    CannyViewerWindow.cpp
    SliceCannyViewerWidget.cpp
)

set(MACOSX_BUNDLE_ICON icon.icns)
set_source_files_properties(${MACOSX_BUNDLE_ICON} PROPERTIES
    MACOSX_PACKAGE_LOCATION Resources
)
if(APPLE)
    list(APPEND srcs ${MACOSX_BUNDLE_ICON})
endif()

set(copyright "Copyright 2023 EduceLab")
string(CONCAT info_string
    "Canny Segment"
    " Version ${PROJECT_VERSION}"
    ", "
    ${copyright}
)

add_executable(${target} ${srcs})

set_target_properties(${target} PROPERTIES
    WIN32_EXECUTABLE ON
    MACOSX_BUNDLE ON
    MACOSX_BUNDLE_BUNDLE_NAME "Canny Segment"
    MACOSX_BUNDLE_GUI_IDENTIFIER "com.educelab.Canny Segment"
    MACOSX_BUNDLE_INFO_STRING ${info_string}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_LONG_VERSION_STRING "Canny Segment Version ${PROJECT_VERSION}"
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION}
    MACOSX_BUNDLE_ICON_FILE ${MACOSX_BUNDLE_ICON}
    MACOSX_BUNDLE_COPYRIGHT ${copyright}
)
if(APPLE)
    set_target_properties(${target} PROPERTIES
        OUTPUT_NAME "CannySegment"
        INSTALL_RPATH "@executable_path/../Frameworks/"
    )
endif()

target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(${target}
    VC::app_support
    VC::core
    VC::gui_support
    VC::meshing
    ${VC_FS_LIB}
    VTK::CommonCore
    VTK::CommonDataModel
    VTK::FiltersGeneral
    VTK::FiltersSources
    Boost::program_options
    opencv_core
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
)

if (${VC_INSTALL_APPS})
    install(
        TARGETS ${target}
        BUNDLE DESTINATION . COMPONENT Programs
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Programs
    )

    qt_generate_deploy_app_script(
        TARGET ${target}
        FILENAME_VARIABLE canny_gui_deploy_script
        NO_UNSUPPORTED_PLATFORM_ERROR
    )
    install(SCRIPT ${canny_gui_deploy_script} COMPONENT Programs)
endif()
