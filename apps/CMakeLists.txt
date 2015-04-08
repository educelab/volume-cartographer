cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Mesher
add_executable(vc_mesher mesher.cpp)
target_link_libraries(vc_mesher ${PCL_LIBRARIES})
target_link_libraries(vc_mesher ${OpenCV_LIBS})
target_link_libraries(vc_mesher meshing)

# Segment
add_executable(vc_segment segment.cpp)
target_link_libraries(vc_segment ${PCL_LIBRARIES})
target_link_libraries(vc_segment ${OpenCV_LIBS})
target_link_libraries(vc_segment volumepkg segmentation)
