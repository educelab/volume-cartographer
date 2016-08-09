# VC-Texture app
set(target VCTexture)

set(srcs
    VCTextureMain.cpp
    Texture_Viewer.cpp
    Segmentations_Viewer.cpp
    Global_Values.cpp
    MainWindow.cpp
    MyThread.cpp
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
        ${target}
        ${CMAKE_BUILD_TYPE}
        " Version"
        ${PROJECT_VERSION}
        ", "
        ${copyright}
    )
    set(MACOSX_BUNDLE_INFO_STRING ${info_string})
    set(MACOSX_BUNDLE_GUI_IDENTIFIER "${target}${CMAKE_BUILD_TYPE}")
    set(MACOSX_BUNDLE_LONG_VERSION_STRING
        "${target}${CMAKE_BUILD_TYPE} Version ${PROJECT_VERSION}")
    set(MACOSX_BUNDLE_BUNDLE_NAME ${target}${CMAKE_BUILD_TYPE})
    set(MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION})
    set(MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION})
    set(MACOSX_BUNDLE_COPYRIGHT ${copyright})
endif()

set(CMAKE_INCLUDE_CURRENT_DIR on)
add_executable(${target} MACOSX_BUNDLE WIN32 ${srcs})
set_target_properties(${target} PROPERTIES
    AUTOMOC on
    AUTOUIC on
)
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        # XXX This should go away - but PCL doesn't specify their targets in a
        # way that lets you do transitive dependencies.
        ${PCL_COMMON_INCLUDE_DIRS}
)
target_link_libraries(${target}
    vc_common
    vc_volumepkg
    vc_texturing
    vc_external
    vc_meshing
    Boost::filesystem
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
)

install(TARGETS ${target}
    BUNDLE DESTINATION . COMPONENT Programs
    RUNTIME DESTINATION bin COMPONENT Programs
)

if (APPLE)
    set(vc_exe "VC-Texture.app")
elseif (WIN32)
    set(vc_exe "VC-Texture.exe")
else ()
    set(vc_exe "VC-Texture")
endif()
set(plugins qcocoa)
set(extra_libs "")
set(extra_dirs ${CMAKE_PREFIX_PATH})
set(request_qt_conf off)
set(plugins_dir "")
set(components Programs)
if(APPLE OR WIN32)
    include(DeployQt5)
    install_qt5_executable(
        ${vc_exe}
        ${plugins}
        ${extra_libs}
        ${extra_dirs}
        ${plugins_dir}
        ${request_qt_conf}
        ${components}
    )
endif()
