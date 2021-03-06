# VC-Texture app
set(target VC-Texture)

set(srcs
    VCTextureMain.cpp
    TextureViewer.cpp
    SegmentationsViewer.cpp
    GlobalValues.cpp
    MainWindow.cpp
    MyThread.cpp
    Rendering.cpp
)

# Set up .app bundle stuff
if (APPLE)
    set(icon vc-texture.icns)
    set_source_files_properties(${icon} PROPERTIES
        MACOSX_PACKAGE_LOCATION Resources
    )
    list(APPEND srcs ${icon})
    # set how it shows up in the Info.plist file
    set(MACOSX_BUNDLE_ICON_FILE ${icon})
    set(copyright "Copyright 2015 University of Kentucky")
    string(CONCAT info_string
        ${target}${CMAKE_BUILD_TYPE}
        " Version ${PROJECT_VERSION}"
        ", "
        ${copyright}
    )
    set(MACOSX_BUNDLE_INFO_STRING ${info_string})
    set(MACOSX_BUNDLE_GUI_IDENTIFIER "${target}")
    set(MACOSX_BUNDLE_LONG_VERSION_STRING
            "${target}-${CMAKE_BUILD_TYPE} Version ${PROJECT_VERSION}")
    set(MACOSX_BUNDLE_BUNDLE_NAME ${target})
    set(MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION})
    set(MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION})
    set(MACOSX_BUNDLE_COPYRIGHT ${copyright})
endif()

add_executable(${target} MACOSX_BUNDLE WIN32 ${srcs})
set_target_properties(${target} PROPERTIES
    AUTOMOC on
    AUTOUIC on
)
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(${target}
    VC::core
    VC::texturing
    VC::meshing
    ${VC_FS_LIB}
    vtkFiltersCore
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    ${OSXSecurity}
)

if (VC_INSTALL_APPS)
    install(TARGETS ${target}
        BUNDLE DESTINATION . COMPONENT Programs
        RUNTIME DESTINATION bin COMPONENT Programs
    )
endif()

if (VC_INSTALL_APPS AND (APPLE OR WIN32))
    qt5_deploy_bundle(${target})
endif()
