cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# autorun MOC when needed
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# build
set(texture_SOURCES main.cpp Texture_Viewer.h Texture_Viewer.cpp Segmentations_Viewer.h Segmentations_Viewer.cpp Global_Values.cpp Global_Values.h MainWindow.cpp MainWindow.h MyThread.cpp MyThread.h)
if(APPLE)
    # Add the app icons to the source files
    set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/vc-texture.icns PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    set(texture_SOURCES ${texture_SOURCES} vc-texture.icns)

    # Add the texture app target
    set(texture_TARGET "VC-Texture")
    add_executable(${texture_TARGET} MACOSX_BUNDLE ${texture_SOURCES})

    # set how it shows up in the Info.plist file
    set_target_properties(${texture_TARGET} PROPERTIES
            MACOSX_BUNDLE_ICON_FILE vc-texture.icns
            MACOSX_BUNDLE_INFO_STRING "${PROJECT_NAME}${DBG_EXTENSION} Version ${PROJECT_VERSION}, Copyright 2015 University of Kentucky."
            MACOSX_BUNDLE_GUI_IDENTIFIER "${PROJECT_NAME}${DBG_EXTENSION}"
            MACOSX_BUNDLE_LONG_VERSION_STRING "${PROJECT_NAME}${DBG_EXTENSION} Version ${PROJECT_VERSION}"
            MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME}${DBG_EXTENSION}
            MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION}
            MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
            MACOSX_BUNDLE_COPYRIGHT "Copyright 2015, University of Kentucky."
            )
else()
    add_executable(${texture_TARGET} ${texture_SOURCES})
endif(APPLE)
target_link_libraries(${texture_TARGET} Qt5::Widgets ${VC_LIBS} ${VC_REQ_LIBS})

# Install targets
install(TARGETS ${texture_TARGET}
        RUNTIME DESTINATION . COMPONENT Programs
        BUNDLE  DESTINATION bin COMPONENT Programs)