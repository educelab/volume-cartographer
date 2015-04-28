cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# only include the headers that needs to be processed by moc
SET(PathGen_SOURCES main.cpp CQtImageViewer.cpp timeUtils.cpp bezierUtils.cpp)
SET(PathGen_HEADERS CQtImageViewer.h)

# invoke moc
QT4_WRAP_CPP(PathGen_HEADERS_MOC ${PathGen_HEADERS})

#Qt include directories
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

# build
add_executable(vc_pathgen ${PathGen_SOURCES} ${PathGen_HEADERS_MOC})

target_link_libraries(vc_pathgen ${VC_LIBS} ${OpenCV_LIBS} ${QT_LIBRARIES} ${EXTRA_LIBS} ${PCL_LIBRARIES})
