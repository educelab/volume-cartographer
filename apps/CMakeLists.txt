cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Texture UI
add_subdirectory(textureUI)

# Mesher
add_executable(vc_mesher mesher.cpp)
target_link_libraries(vc_mesher ${VC_LIBS} ${VC_REQ_LIBS})

# Segment
add_executable(vc_segment segment.cpp)
target_link_libraries(vc_segment ${VC_LIBS} ${VC_REQ_LIBS})

# Metadata Editor
add_executable(vc_metaedit metadata.cpp)
target_link_libraries(vc_metaedit ${VC_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# Install targets
install(TARGETS vc_mesher vc_segment vc_metaedit
        RUNTIME DESTINATION bin COMPONENT Programs)

# Mesh Editor
# only include the headers that needs to be processed by moc
SET(meshEditor_SOURCES  meshEditor/Ply/base.cpp meshEditor/Ply/header.cpp meshEditor/Ply/io.cpp meshEditor/Ply/object.cpp
                        meshEditor/Ply/unknown.cpp meshEditor/main.cpp meshEditor/CSimpleNumEditBox.cpp meshEditor/C2DView.cpp
                        meshEditor/C3DView.cpp meshEditor/CMesh.cpp meshEditor/CMeshGL.cpp meshEditor/CMeshIO.cpp
                        meshEditor/CWindow.cpp meshEditor/CXCurve.cpp meshEditor/UDataManipulateUtils.cpp meshEditor/UObjHelper.cpp
                        meshEditor/CPlyHelper.cpp)

SET(meshEditor_HEADERS  meshEditor/Ply/base.h meshEditor/Ply/header.h meshEditor/Ply/io.h meshEditor/Ply/object.h
                        meshEditor/Ply/unknown.h meshEditor/CSimpleNumEditBox.h meshEditor/C2DView.h meshEditor/C3DObj.h
                        meshEditor/C3DView.h meshEditor/CLinkedList.h meshEditor/CMeshGL.h meshEditor/CMesh.h meshEditor/CMeshIO.h
                        meshEditor/CVec.h meshEditor/CPoint.h meshEditor/CWindow.h meshEditor/CXCurve.h meshEditor/HBase.h
                        meshEditor/mathUtils.h meshEditor/UDataManipulateUtils.h meshEditor/UObjHelper.h meshEditor/CPlyHelper.h)

# invoke moc
QT5_WRAP_CPP(meshEditor_HEADERS_MOC ${meshEditor_HEADERS})

add_executable(vc_meshEditor ${meshEditor_SOURCES} ${meshEditor_HEADERS_MOC})
target_link_libraries(vc_meshEditor ${VC_LIBS} ${VC_REQ_LIBS} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL ${OPENGL_LIBRARIES} ${QT_LIBRARIES} ${EXTRA_LIBS})

# Install targets
install(TARGETS vc_meshEditor
        RUNTIME DESTINATION bin COMPONENT Demos)

# VCNewUI - I'm sorry for everything below this line. - SP
# only include the headers that needs to be processed by moc
SET(VCNewUI_SOURCES     meshEditor/Ply/base.cpp meshEditor/Ply/header.cpp meshEditor/Ply/io.cpp meshEditor/Ply/object.cpp
                        meshEditor/Ply/unknown.cpp VCNewUI/main.cpp VCNewUI/CSimpleNumEditBox.cpp VCNewUI/CMesh.cpp
                        VCNewUI/CWindow.cpp VCNewUI/CXCurve.cpp VCNewUI/UDataManipulateUtils.cpp VCNewUI/UObjHelper.cpp
                        VCNewUI/CPlyHelper.cpp VCNewUI/CVolumeViewer.cpp VCNewUI/CVolumeViewerWithCurve.cpp VCNewUI/CBSpline.cpp
                        VCNewUI/CBezierCurve.cpp)

SET(VCNewUI_HEADERS     meshEditor/Ply/base.h meshEditor/Ply/header.h meshEditor/Ply/io.h meshEditor/Ply/object.h
                        meshEditor/Ply/unknown.h VCNewUI/CSimpleNumEditBox.h VCNewUI/CLinkedList.h VCNewUI/CMesh.h VCNewUI/CVec.h
                        VCNewUI/CPoint.h VCNewUI/CWindow.h VCNewUI/CXCurve.h VCNewUI/HBase.h VCNewUI/mathUtils.h
                        VCNewUI/UDataManipulateUtils.h VCNewUI/UObjHelper.h VCNewUI/CPlyHelper.h VCNewUI/CVolumeViewer.h
                        VCNewUI/CVolumeViewerWithCurve.h VCNewUI/CBSpline.h VCNewUI/CBezierCurve.h)

SET(VCNewUI_UIS VCNewUI/VCMain.ui)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# invoke moc
QT5_WRAP_UI(VCNewUI_UIS_HDS ${VCNewUI_UIS})
QT5_WRAP_CPP(VCNewUI_HEADERS_MOC ${VCNewUI_HEADERS})

if(APPLE AND BUILD_OSX_APPS)
    # set where in the bundle to put the icns file
    SET_SOURCE_FILES_PROPERTIES(${CMAKE_CURRENT_SOURCE_DIR}/VCNewUI/vc-logo.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    # include the icns file in the target
    SET(VCNewUI_SOURCES ${VCNewUI_SOURCES} VCNewUI/vc-logo.icns)
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

target_link_libraries(VC ${VC_LIBS} ${VC_REQ_LIBS} Qt5::Core Qt5::Gui Qt5::Widgets ${OPENGL_LIBRARIES} ${QT_LIBRARIES} ${EXTRA_LIBS})
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
