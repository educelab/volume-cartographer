cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
# VC.app

# autorun MOC when needed
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(VIEWER_APP_NAME "VC Mapping Viewer")

SET(mapView_SOURCES     main.cpp mWindow.cpp)
SET(mapView_HEADERS     mWindow.h)

# Always build .app if APPLE
if(APPLE)
    # Add the icon to the app bundle
    SET_SOURCE_FILES_PROPERTIES(${CMAKE_CURRENT_SOURCE_DIR}/vc-logo.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    #SET(mapView_SOURCES ${mapView_SOURCES} vc-logo.icns)

    # set how it shows up in the Info.plist file
    #set(MACOSX_BUNDLE_ICON_FILE "vc-logo.icns")
    set(MACOSX_BUNDLE_INFO_STRING "${VIEWER_APP_NAME}${DBG_EXTENSION} Version ${PROJECT_VERSION}, Copyright 2015 University of Kentucky.")
    set(MACOSX_BUNDLE_GUI_IDENTIFIER "${VIEWER_APP_NAME}${DBG_EXTENSION}")
    set(MACOSX_BUNDLE_LONG_VERSION_STRING "${VIEWER_APP_NAME}${DBG_EXTENSION} Version ${PROJECT_VERSION}")
    set(MACOSX_BUNDLE_BUNDLE_NAME ${VIEWER_APP_NAME}${DBG_EXTENSION})
    set(MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION})
    set(MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION})
    set(MACOSX_BUNDLE_COPYRIGHT "Copyright 2015, University of Kentucky.")
endif(APPLE)

add_executable(vc_mappingViewer MACOSX_BUNDLE WIN32 ${mapView_SOURCES} ${mapView_HEADERS})
target_link_libraries(vc_mappingViewer ${VC_LIBS} ${VC_REQ_LIBS} Qt5::Core Qt5::Gui Qt5::Widgets ${OPENGL_LIBRARIES} ${EXTRA_LIBS})

set(mapView_EXECUTABLE "vc_mappingViewer")
if(APPLE)
    set(VC_EXECUTABLE "vc_mappingViewer.app")
elseif(WIN32)
    set(VC_EXECUTABLE "vc_mappingViewer.exe")
endif()

set(mapView_PLUGINS qcocoa)
#set(mapView_EXTRA_LIBS)
#set(mapView_PLUGINS_DIR)
#set(mapView_REQUEST_QT_CONF)
set(mapView_SEARCH_DIRS "${CMAKE_PREFIX_PATH}")

install(TARGETS vc_mappingViewer
        BUNDLE DESTINATION . COMPONENT Programs
        RUNTIME DESTINATION bin COMPONENT Programs)

if(APPLE OR WIN32)
    include(DeployQt5)
    install_qt5_executable("${mapView_EXECUTABLE}" "${mapView_PLUGINS}" "${mapView_EXTRA_LIBS}" "${mapView_SEARCH_DIRS}" "${mapView_REQUEST_QT_CONF}" "${mapView_PLUGINS_DIR}" "Programs")
endif()
