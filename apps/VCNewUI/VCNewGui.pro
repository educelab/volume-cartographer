HEADERS = CMesh.h \
          CXCurve.h \
          CWindow.h \
    UDataManipulateUtils.h \
    UObjHelper.h \
          mathUtils.h \
    ../../meshing/orderedPCDMesher.h \
    ../meshEditor/Ply/base.h \
    ../meshEditor/Ply/header.h \
    ../meshEditor/Ply/io.h \
    ../meshEditor/Ply/object.h \
    ../meshEditor/Ply/unknown.h \
    ../../volumepkg/picojson.h \
    ../../volumepkg/volumepkg.h \
    ../../volumepkg/volumepkgcfg.h \
    HBase.h \
    ../meshEditor/objTester/list.h \
    ../meshEditor/objTester/obj_parser.h \
    ../meshEditor/objTester/objLoader.h \
    ../meshEditor/objTester/string_extra.h \
    VCNewGuiHeader.h \
    CPlyHelper.h \
    CBezierCurve.h \
    CBSpline.h \
    CVectorN.h \
    CMatrixMN.h \
    CVolumeViewer.h \
    CSimpleNumEditBox.h \
    CVolumeViewerWithCurve.h
SOURCES = CMesh.cpp \
          CXCurve.cpp \
          CWindow.cpp \
    UDataManipulateUtils.cpp \
    UObjHelper.cpp \
          main.cpp \
    ../../meshing/orderedPCDMesher.cpp \
    ../meshEditor/Ply/base.cpp \
    ../meshEditor/Ply/header.cpp \
    ../meshEditor/Ply/io.cpp \
    ../meshEditor/Ply/object.cpp \
    ../meshEditor/Ply/unknown.cpp \
    ../../volumepkg/volumepkg.cpp \
    ../../volumepkg/volumepkgcfg.cpp \
    ../meshEditor/objTester/list.cpp \
    ../meshEditor/objTester/obj_parser.cpp \
    ../meshEditor/objTester/objLoader.cpp \
    ../meshEditor/objTester/string_extra.cpp \
    CPlyHelper.cpp \
    CBezierCurve.cpp \
    CBSpline.cpp \
    CVolumeViewer.cpp \
    CSimpleNumEditBox.cpp \
    CVolumeViewerWithCurve.cpp

INCLUDEPATH += "/usr/include/"
INCLUDEPATH += "../meshEditor/objTester/"
INCLUDEPATH += "../meshEditor/"
INCLUDEPATH += "/usr/include/pcl-1.7/"
INCLUDEPATH += "/usr/include/eigen3/"
INCLUDEPATH += "../../"
INCLUDEPATH += "../../meshing/"

LIBS += -L"/usr/lib/x84_64-linux-gnu/" -lopencv_core -lopencv_highgui -lopencv_imgproc -lboost_system -lboost_filesystem -lpcl_io -lpcl_common
QT += widgets opengl
QMAKE_CXXFLAGS += -std=c++11

FORMS += \
    VCMain.ui
