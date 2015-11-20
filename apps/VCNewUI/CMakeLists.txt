cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
# VC.app

# autorun MOC when needed
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# only include the headers that needs to be processed by moc
SET(VCNewUI_SOURCES     CSimpleNumEditBox.cpp main.cpp CMesh.cpp
                        CWindow.cpp CXCurve.cpp UDataManipulateUtils.cpp
                        CVolumeViewer.cpp CVolumeViewerWithCurve.cpp CBSpline.cpp
                        CBezierCurve.cpp)


SET(VCNewUI_HEADERS     CSimpleNumEditBox.h CMesh.h CVectorN.h CMatrixMN.h UVecMatOperations.h
                        CWindow.h CXCurve.h HBase.h mathUtils.h
                        UDataManipulateUtils.h CVolumeViewer.h
                        CVolumeViewerWithCurve.h CBSpline.h CBezierCurve.h)

# Always build .app if APPLE
if(APPLE)
  # Add the icon to the app bundle
  SET_SOURCE_FILES_PROPERTIES(${CMAKE_CURRENT_SOURCE_DIR}/vc-logo.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
  SET(VCNewUI_SOURCES ${VCNewUI_SOURCES} vc-logo.icns)

  # set how it shows up in the Info.plist file
  set(MACOSX_BUNDLE_ICON_FILE "vc-logo.icns")
  set(MACOSX_BUNDLE_INFO_STRING "${PROJECT_NAME}${DBG_EXTENSION} Version ${PROJECT_VERSION}, Copyright 2015 University of Kentucky.")
  set(MACOSX_BUNDLE_GUI_IDENTIFIER "${PROJECT_NAME}${DBG_EXTENSION}")
  set(MACOSX_BUNDLE_LONG_VERSION_STRING "${PROJECT_NAME}${DBG_EXTENSION} Version ${PROJECT_VERSION}")
  set(MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME}${DBG_EXTENSION})
  set(MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION})
  set(MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION})
  set(MACOSX_BUNDLE_COPYRIGHT "Copyright 2015, University of Kentucky.")
endif(APPLE)

add_executable(VC MACOSX_BUNDLE WIN32 ${VCNewUI_SOURCES} ${VCNewUI_HEADERS})
target_link_libraries(VC ${VC_LIBS} ${VC_REQ_LIBS} Qt4::QtOpenGL ${OPENGL_LIBRARIES} ${QT_LIBRARIES} ${EXTRA_LIBS})

set(VC_EXECUTABLE "VC")
if(APPLE)
    set(VC_EXECUTABLE "VC.app")
elseif(WIN32)
    set(VC_EXECUTABLE "VC.exe")
endif()

#set(PLUGINS)
#set(LIBS)
#set(PLUGINS_DIR)
#set(REQUEST_QT_CONF)
set(DIRS "${CMAKE_PREFIX_PATH}")

install(TARGETS VC
        BUNDLE DESTINATION . COMPONENT Programs
        RUNTIME DESTINATION bin COMPONENT Programs)

if(APPLE OR WIN32)
    include(DeployQt4)
    install_qt4_executable("${VC_EXECUTABLE}" "" "" "${DIRS}" "" "" "Programs")
endif()
