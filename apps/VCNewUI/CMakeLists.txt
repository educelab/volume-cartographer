cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# VCNewUI - I'm sorry for everything below this line. - SP
# only include the headers that needs to be processed by moc
SET(VCNewUI_SOURCES     CSimpleNumEditBox.cpp main.cpp CMesh.cpp
                        CWindow.cpp CXCurve.cpp UDataManipulateUtils.cpp
                        CVolumeViewer.cpp CVolumeViewerWithCurve.cpp CBSpline.cpp
                        CBezierCurve.cpp)


SET(VCNewUI_HEADERS     CSimpleNumEditBox.h CMesh.h CVectorN.h CMatrixMN.h UVecMatOperations.h
                        CWindow.h CXCurve.h HBase.h mathUtils.h
                        UDataManipulateUtils.h CVolumeViewer.h
                        CVolumeViewerWithCurve.h CBSpline.h CBezierCurve.h)

SET(VCNewUI_UIS VCMain.ui)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# invoke moc
QT4_WRAP_UI(VCNewUI_UIS_HDS ${VCNewUI_UIS})
QT4_WRAP_CPP(VCNewUI_HEADERS_MOC ${VCNewUI_HEADERS})

if(APPLE AND BUILD_OSX_APPS)
    # set where in the bundle to put the icns file
    SET_SOURCE_FILES_PROPERTIES(${CMAKE_CURRENT_SOURCE_DIR}/vc-logo.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    # include the icns file in the target
    SET(VCNewUI_SOURCES ${VCNewUI_SOURCES} vc-logo.icns)
    add_executable(VC MACOSX_BUNDLE ${VCNewUI_UIS_HDS} ${VCNewUI_SOURCES} ${VCNewUI_HEADERS_MOC})

    # set how it shows up in the Info.plist file
    set_target_properties(VC PROPERTIES
            MACOSX_BUNDLE_ICON_FILE vc-logo.icns
            MACOSX_BUNDLE_INFO_STRING "${PROJECT_NAME}${DBG_EXTENSION} Version ${PROJECT_VERSION}, Copyright 2015 University of Kentucky."
            MACOSX_BUNDLE_GUI_IDENTIFIER "${PROJECT_NAME}${DBG_EXTENSION}"
            MACOSX_BUNDLE_LONG_VERSION_STRING "${PROJECT_NAME}${DBG_EXTENSION} Version ${PROJECT_VERSION}"
            MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME}${DBG_EXTENSION}
            MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION}
            MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
            MACOSX_BUNDLE_COPYRIGHT "Copyright 2015, University of Kentucky."
            )
else()
    add_executable(VC ${VCNewUI_UIS_HDS} ${VCNewUI_SOURCES} ${VCNewUI_HEADERS_MOC})
endif(APPLE AND BUILD_OSX_APPS)

target_link_libraries(VC ${VC_LIBS} ${VC_REQ_LIBS} Qt4::QtOpenGL ${OPENGL_LIBRARIES} ${QT_LIBRARIES} ${EXTRA_LIBS})
INSTALL(TARGETS VC
        BUNDLE DESTINATION . COMPONENT Programs
        RUNTIME DESTINATION bin COMPONENT Programs)

if(BUILD_OSX_APPS)
    SET(plugin_dest_dir VC.app/Contents/MacOS)
    SET(qtconf_dest_dir VC.app/Contents/Resources)
    SET(APPS "\${CMAKE_INSTALL_PREFIX}/VC.app")

    #--------------------------------------------------------------------------------
    # Install needed Qt plugins by copying directories from the qt installation
    # One can cull what gets copied by using 'REGEX "..." EXCLUDE'
    INSTALL(DIRECTORY "${QT_PLUGINS_DIR}/imageformats" DESTINATION ${plugin_dest_dir}/plugins COMPONENT Runtime)

    #--------------------------------------------------------------------------------
    # install a qt.conf file
    # this inserts some cmake code into the install script to write the file
    INSTALL(CODE "
    file(WRITE \"\${CMAKE_INSTALL_PREFIX}/${qtconf_dest_dir}/qt.conf\" \"\")
    " COMPONENT Runtime)

    #--------------------------------------------------------------------------------
    # Use BundleUtilities to get all other dependencies for the application to work.
    # It takes a bundle or executable along with possible plugins and inspects it
    # for dependencies.  If they are not system dependencies, they are copied.

    # directories to look for dependencies
    SET(DIRS ${QT_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

    # Now the work of copying dependencies into the bundle/package
    # The quotes are escaped and variables to use at install time have their $ escaped
    # An alternative is the do a configure_file() on a script and use install(SCRIPT  ...).
    # Note that the image plugins depend on QtSvg and QtXml, and it got those copied
    # over.
    INSTALL(CODE "
    file(GLOB_RECURSE QTPLUGINS \"\${CMAKE_INSTALL_PREFIX}/${plugin_dest_dir}/plugins/*${CMAKE_SHARED_LIBRARY_SUFFIX}\")
    set(BU_CHMOD_BUNDLE_ITEMS ON)
    include(BundleUtilities)
    fixup_bundle(\"${APPS}\" \"\${QTPLUGINS}\" \"${DIRS}\" COMPONENT Runtime)")
    set(CPACK_BINARY_DRAGNDROP ON)
endif(BUILD_OSX_APPS)