cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_CXX_FLAGS "-std=c++11")

PROJECT(PathGen)
FIND_PACKAGE(Qt4 REQUIRED)
find_package(OpenCV REQUIRED)

# only include the headers that needs to be processed by moc
SET(PathGen_SOURCES main.cpp CQtImageViewer.cpp timeUtils.cpp bezierUtils.cpp)
SET(PathGen_HEADERS CQtImageViewer.h)

# invoke moc
QT4_WRAP_CPP(PathGen_HEADERS_MOC ${PathGen_HEADERS})

#Qt include directories
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

add_subdirectory(./../volumepkg volumepkg)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${VOLUMEPKG_INCLUDE_DIR})

# build
add_executable(vc_pathgen ${PathGen_SOURCES} ${PathGen_HEADERS_MOC})

target_link_libraries(vc_pathgen volumepkg ${OpenCV_LIBS} ${QT_LIBRARIES})
