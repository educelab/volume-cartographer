cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# volpkg
add_executable(vc_volpkg_example volpkg.cpp)
target_link_libraries(vc_volpkg_example ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_objExporterExample objExporterExample.cpp)
target_link_libraries(vc_objExporterExample ${ITK_LIBRARIES} ${VC_LIBS} ${VC_REQ_LIBS})