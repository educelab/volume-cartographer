cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# autorun MOC when needed
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# build
set(texture_SOURCES main.cpp Texture_Viewer.h Texture_Viewer.cpp
                    Segmentations_Viewer.h Segmentations_Viewer.cpp
                    Global_Values.cpp Global_Values.h MainWindow.cpp MainWindow.h
                    MyThread.cpp MyThread.h)

# Always build .app if APPLE
if(APPLE)
  # Add the icon to the app bundle
  SET_SOURCE_FILES_PROPERTIES(${CMAKE_CURRENT_SOURCE_DIR}/vc-texture.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
  SET(texture_SOURCES ${texture_SOURCES} vc-texture.icns)

  # set how it shows up in the Info.plist file
  set(MACOSX_BUNDLE_ICON_FILE "vc-texture.icns")
  set(MACOSX_BUNDLE_INFO_STRING "VC Texture${DBG_EXTENSION} Version ${PROJECT_VERSION}, Copyright 2015 University of Kentucky.")
  set(MACOSX_BUNDLE_GUI_IDENTIFIER "VC Texture${DBG_EXTENSION}")
  set(MACOSX_BUNDLE_LONG_VERSION_STRING "VC Texture${DBG_EXTENSION} Version ${PROJECT_VERSION}")
  set(MACOSX_BUNDLE_BUNDLE_NAME "VC Texture${DBG_EXTENSION}")
  set(MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION})
  set(MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION})
  set(MACOSX_BUNDLE_COPYRIGHT "Copyright 2015, University of Kentucky.")
endif(APPLE)

add_executable(VC-Texture MACOSX_BUNDLE WIN32 ${texture_SOURCES})
target_link_libraries(VC-Texture Qt5::Widgets ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

set(Texture_EXECUTABLE "VC-Texture")
if(APPLE)
    set(Texture_EXECUTABLE "VC-Texture.app")
elseif(WIN32)
    set(Texture_EXECUTABLE "VC-Texture.exe")
endif()

set(Texture_PLUGINS qcocoa)
#set(Texture_EXTRA_LIBS)
#set(Texture_PLUGINS_DIR)
#set(Texture_REQUEST_QT_CONF)
set(Texture_SEARCH_DIRS "${CMAKE_PREFIX_PATH}")

install(TARGETS VC-Texture
        BUNDLE DESTINATION . COMPONENT Programs
        RUNTIME DESTINATION bin COMPONENT Programs)

if(APPLE OR WIN32)
    include(DeployQt5)
    install_qt5_executable("${Texture_EXECUTABLE}" "${Texture_PLUGINS}" "${Texture_EXTRA_LIBS}" "${Texture_SEARCH_DIRS}" "${Texture_REQUEST_QT_CONF}" "${Texture_PLUGINS_DIR}" "Programs")
endif()
