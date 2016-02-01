cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

###########
# volpkg  #
###########
add_executable(vc_volpkg_example volpkg.cpp)
target_link_libraries(vc_volpkg_example ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_objExporterExample objExporterExample.cpp )
target_link_libraries(vc_objExporterExample ${VC_LIBS} ${VC_REQ_LIBS})

############
#  meshing #
############

add_executable(vc_itk2vtkExample itk2vtkExample.cpp)
target_link_libraries(vc_itk2vtkExample ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

add_executable(vc_greedyExample greedyExample.cpp)
target_link_libraries(vc_greedyExample ${VC_LIBS} ${VC_REQ_LIBS})
add_executable(vc_uvMapExample uvMapExample.cpp)
target_link_libraries(vc_uvMapExample ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_poissonExample poissonExample.cpp)
target_link_libraries(vc_poissonExample ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_plyExporterExample plyExporterExample.cpp)
target_link_libraries(vc_plyExporterExample ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_resamplePointCloudExample resamplePointCloudExample.cpp)
target_link_libraries(vc_resamplePointCloudExample ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_orderedPCDExample orderedPCDExample.cpp)
target_link_libraries(vc_orderedPCDExample ${VC_LIBS} ${VC_REQ_LIBS})

file(COPY ../testing/test_data/meshing/vtk.ply DESTINATION ${CMAKE_BINARY_DIR}/examples)
file(COPY ../testing/test_data/meshing/itk.obj DESTINATION ${CMAKE_BINARY_DIR}/examples)
add_executable(vc_parsingHelpers parsingHelperExample.cpp ../testing/parsingHelpers.cpp ../testing/parsingHelpers.h)
target_link_libraries(vc_parsingHelpers ${VC_LIBS} ${VC_REQ_LIBS} )

add_executable(vc_rayTraceExample rayTraceExample.cpp)
target_link_libraries(vc_rayTraceExample ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_smoothingExample smoothingExample.cpp)
target_link_libraries(vc_smoothingExample ${VC_LIBS} ${VC_REQ_LIBS})