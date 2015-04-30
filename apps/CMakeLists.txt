cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Mesher
add_executable(vc_mesher mesher.cpp)
target_link_libraries(vc_mesher ${PCL_LIBRARIES})
target_link_libraries(vc_mesher ${OpenCV_LIBS})
target_link_libraries(vc_mesher ${VC_LIBS})

# Segment
add_executable(vc_segment segment.cpp)
target_link_libraries(vc_segment ${PCL_LIBRARIES})
target_link_libraries(vc_segment ${OpenCV_LIBS})
target_link_libraries(vc_segment ${VC_LIBS})

# Mesh Editor
# only include the headers that needs to be processed by moc
SET(meshEditor_SOURCES meshEditor/Ply/base.cpp meshEditor/Ply/header.cpp meshEditor/Ply/io.cpp meshEditor/Ply/object.cpp meshEditor/Ply/unknown.cpp meshEditor/objTester/list.cpp meshEditor/objTester/objLoader.cpp meshEditor/objTester/obj_parser.cpp meshEditor/objTester/string_extra.cpp meshEditor/main.cpp meshEditor/CSimpleNumEditBox.cpp meshEditor/C2DView.cpp meshEditor/C3DView.cpp meshEditor/CMesh.cpp meshEditor/CMeshGL.cpp meshEditor/CMeshIO.cpp meshEditor/CWindow.cpp meshEditor/CXCurve.cpp meshEditor/UDataManipulateUtils.cpp meshEditor/UObjHelper.cpp meshEditor/CPlyHelper.cpp)
SET(meshEditor_HEADERS meshEditor/Ply/base.h meshEditor/Ply/header.h meshEditor/Ply/io.h meshEditor/Ply/object.h meshEditor/Ply/unknown.h meshEditor/objTester/list.h meshEditor/objTester/objLoader.h meshEditor/objTester/obj_parser.h meshEditor/objTester/string_extra.h meshEditor/CSimpleNumEditBox.h meshEditor/C2DView.h meshEditor/C3DObj.h meshEditor/C3DView.h meshEditor/CLinkedList.h meshEditor/CMeshGL.h meshEditor/CMesh.h meshEditor/CMeshIO.h meshEditor/CVec.h meshEditor/CPoint.h meshEditor/CWindow.h meshEditor/CXCurve.h meshEditor/HBase.h meshEditor/mathUtils.h meshEditor/UDataManipulateUtils.h meshEditor/UObjHelper.h meshEditor/CPlyHelper.h)
# invoke moc
QT4_WRAP_CPP(meshEditor_HEADERS_MOC ${meshEditor_HEADERS})
#Qt include directories
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})
add_executable(vc_meshEditor ${meshEditor_SOURCES} ${meshEditor_HEADERS_MOC})
target_link_libraries(vc_meshEditor ${VC_LIBS} Qt4::QtOpenGL ${OPENGL_LIBRARIES} ${OpenCV_LIBS} ${QT_LIBRARIES} ${EXTRA_LIBS} ${PCL_LIBRARIES})

# VCNewUI
# only include the headers that needs to be processed by moc
SET(VCNewUI_SOURCES meshEditor/Ply/base.cpp meshEditor/Ply/header.cpp meshEditor/Ply/io.cpp meshEditor/Ply/object.cpp meshEditor/Ply/unknown.cpp meshEditor/objTester/list.cpp meshEditor/objTester/objLoader.cpp meshEditor/objTester/obj_parser.cpp meshEditor/objTester/string_extra.cpp VCNewUI/main.cpp VCNewUI/CSimpleNumEditBox.cpp VCNewUI/CMesh.cpp VCNewUI/CWindow.cpp VCNewUI/CXCurve.cpp VCNewUI/UDataManipulateUtils.cpp VCNewUI/UObjHelper.cpp VCNewUI/CPlyHelper.cpp VCNewUI/CVolumeViewer.cpp VCNewUI/CVolumeViewerWithCurve.cpp VCNewUI/CBSpline.cpp VCNewUI/CBezierCurve.cpp)
SET(VCNewUI_HEADERS meshEditor/Ply/base.h meshEditor/Ply/header.h meshEditor/Ply/io.h meshEditor/Ply/object.h meshEditor/Ply/unknown.h meshEditor/objTester/list.h meshEditor/objTester/objLoader.h meshEditor/objTester/obj_parser.h meshEditor/objTester/string_extra.h VCNewUI/CSimpleNumEditBox.h VCNewUI/CLinkedList.h VCNewUI/CMesh.h VCNewUI/CVec.h VCNewUI/CPoint.h VCNewUI/CWindow.h VCNewUI/CXCurve.h VCNewUI/HBase.h VCNewUI/mathUtils.h VCNewUI/UDataManipulateUtils.h VCNewUI/UObjHelper.h VCNewUI/CPlyHelper.h VCNewUI/CVolumeViewer.h VCNewUI/CVolumeViewerWithCurve.h VCNewUI/CBSpline.h VCNewUI/CBezierCurve.h)
SET(VCNewUI_UIS VCNewUI/VCMain.ui)
# invoke moc
QT4_WRAP_UI(VCNewUI_UIS_HDS ${VCNewUI_UIS})
QT4_WRAP_CPP(VCNewUI_HEADERS_MOC ${VCNewUI_HEADERS})
#Qt include directories
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})
add_executable(vc_VCNewUI ${VCNewUI_UIS_HDS} ${VCNewUI_SOURCES} ${VCNewUI_HEADERS_MOC})
include_directories(${PCL_INCLUDE_DIRS} ../meshing/ meshEditor/ meshEditor/objTester/ ../)
target_link_libraries(vc_VCNewUI ${VC_LIBS} Qt4::QtOpenGL ${OPENGL_LIBRARIES} ${OpenCV_LIBS} ${QT_LIBRARIES} ${EXTRA_LIBS} ${PCL_LIBRARIES} pthread ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
