cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# autorun MOC when needed
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# build
add_executable(vc_textureUI main.cpp Texture_Viewer.h Texture_Viewer.cpp Segmentations_Viewer.h Segmentations_Viewer.cpp DefaultValues.cpp DefaultValues.h)
target_link_libraries(vc_textureUI Qt5::Widgets ${VC_LIBS} ${VC_REQ_LIBS})