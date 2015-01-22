HEADERS = CMesh.h \
          CXCurve.h \
          C3DView.h \
          C2DView.h \
          CWindow.h \
          C3DObj.h \
    UDataManipulateUtils.h \
    UObjHelper.h \
          CMeshGL.h \
          mathUtils.h \
    CMeshIO.h \
    Ply/base.h \
    Ply/header.h \
    Ply/io.h \
    Ply/object.h \
    Ply/unknown.h \
    volumepkg/picojson.h \
    volumepkg/volumepkg.h \
    volumepkg/volumepkgcfg.h \
    HBase.h \
    objTester/objTester/list.h \
    objTester/objTester/obj_parser.h \
    objTester/objTester/objLoader.h \
    objTester/objTester/string_extra.h \
    MeshEditorHeader.h
SOURCES = CMesh.cpp \
          CXCurve.cpp \
          C3DView.cpp \
          C2DView.cpp \
          CWindow.cpp \
    UDataManipulateUtils.cpp \
    UObjHelper.cpp \
          CMeshGL.cpp \
          main.cpp \
    CMeshIO.cpp \
    Ply/base.cpp \
    Ply/header.cpp \
    Ply/io.cpp \
    Ply/object.cpp \
    Ply/unknown.cpp \
    volumepkg/volumepkg.cpp \
    volumepkg/volumepkgcfg.cpp \
    objTester/objTester/list.cpp \
    objTester/objTester/obj_parser.cpp \
    objTester/objTester/objLoader.cpp \
    objTester/objTester/string_extra.cpp

INCLUDEPATH += "/usr/include/"
INCLUDEPATH += "./objTester/objTester"

LIBS += -L"/usr/lib/x84_64-linux-gnu/" -lopencv_core -lopencv_highgui -lopencv_imgproc
QT += widgets opengl
QMAKE_CXXFLAGS += -std=c++11
