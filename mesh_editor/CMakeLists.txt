cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_CXX_FLAGS "-std=c++11")

FIND_PACKAGE(Qt4 REQUIRED)
find_package(OpenCV REQUIRED)

IF(APPLE)
FIND_LIBRARY(SECURITY_LIBRARY Security)
SET(EXTRA_LIBS ${SECURITY_LIBRARY})
ENDIF(APPLE)

# only include the headers that needs to be processed by moc
SET(meshEditor_SOURCES Ply/base.cpp Ply/header.cpp Ply/io.cpp Ply/object.cpp Ply/unknown.cpp objTester/list.cpp objTester/objLoader.cpp objTester/obj_parser.cpp objTester/string_extra.cpp main.cpp C2DView.cpp C3DView.cpp CMesh.cpp CMeshGL.cpp CMeshIO.cpp CWindow.cpp CXCurve.cpp UDataManipulateUtils.cpp UObjHelper.cpp)
SET(meshEditor_HEADERS Ply/base.h Ply/header.h Ply/io.h Ply/object.h Ply/unknown.h objTester/list.h objTester/objLoader.h objTester/obj_parser.h objTester/string_extra.h C2DView.h C3DObj.h C3DView.h CLinkedList.h CMeshGL.h CMesh.h CMeshIO.h CVec.h CPoint.h CWindow.h CXCurve.h HBase.h mathUtils.h UDataManipulateUtils.h UObjHelper.h)

# invoke moc
QT4_WRAP_CPP(meshEditor_HEADERS_MOC ${meshEditor_HEADERS})

#Qt include directories
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

add_subdirectory(./../volumepkg volumepkg)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${VOLUMEPKG_INCLUDE_DIR})

# build
add_executable(vc_meshEditor ${meshEditor_SOURCES} ${meshEditor_HEADERS_MOC})

target_link_libraries(vc_meshEditor volumepkg -lQtOpenGL -lGL ${OpenCV_LIBS} ${QT_LIBRARIES} ${EXTRA_LIBS})
