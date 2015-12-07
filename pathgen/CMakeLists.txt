cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# only include the headers that needs to be processed by moc
SET(PathGen_SOURCES main.cpp CQtImageViewer.cpp timeUtils.cpp bezierUtils.cpp)
SET(PathGen_HEADERS CQtImageViewer.h)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# invoke moc
QT5_WRAP_CPP(PathGen_HEADERS_MOC ${PathGen_HEADERS})

# build
add_executable(vc_pathgen ${PathGen_HEADERS_MOC} ${PathGen_SOURCES})
target_link_libraries(vc_pathgen ${VC_LIBS} ${VC_REQ_LIBS} Qt5::Widgets Qt5::PrintSupport ${EXTRA_LIBS})

# Install targets
install(TARGETS vc_pathgen
        RUNTIME DESTINATION bin COMPONENT Programs)
