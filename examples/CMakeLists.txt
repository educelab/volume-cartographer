cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_executable(vc_cone_example coneExample.cpp)
target_link_libraries(vc_cone_example ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

###########
# volpkg  #
###########
add_executable(vc_volpkg_example volpkg.cpp)
target_link_libraries(vc_volpkg_example ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_objExporterExample objExporterExample.cpp )
target_link_libraries(vc_objExporterExample ${VC_LIBS} ${VC_REQ_LIBS})

############
#  common  #
############
add_executable(vc_objWriterExample ObjWriterExample.cpp)
target_link_libraries(vc_objWriterExample ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

############
#  meshing #
############

file(COPY ../testing/test_data/meshing/vtk.ply DESTINATION ${CMAKE_BINARY_DIR}/examples)
file(COPY ../testing/test_data/meshing/itk.obj DESTINATION ${CMAKE_BINARY_DIR}/examples)
add_executable(vc_parsingHelpers parsingHelpersExample.cpp ../testing/parsingHelpers.cpp ../testing/parsingHelpers.h)
target_link_libraries(vc_parsingHelpers ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

add_executable(vc_itk2vtkExample itk2vtkExample.cpp)
target_link_libraries(vc_itk2vtkExample ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

add_executable(vc_greedyExample greedyProjectionMeshingExample.cpp)
target_link_libraries(vc_greedyExample ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

add_executable(vc_uvMapExample uvMapExample.cpp)
target_link_libraries(vc_uvMapExample ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_plyExporterExample plyExporterExample.cpp)
target_link_libraries(vc_plyExporterExample ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_resamplePointCloudExample resamplePointCloudExample.cpp)
target_link_libraries(vc_resamplePointCloudExample ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

add_executable(vc_orderedPCDExample orderedPCDMesherExample.cpp)
target_link_libraries(vc_orderedPCDExample ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

add_executable(vc_textureExample textureExample.cpp)
target_link_libraries(vc_textureExample ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_textureFlattened textureFlattened.cpp)
target_link_libraries(vc_textureFlattened ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

add_executable(vc_rayTraceExample rayTraceExample.cpp)
target_link_libraries(vc_rayTraceExample ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

add_executable(vc_scaleMeshExample scaleMeshExample.cpp)
target_link_libraries(vc_scaleMeshExample ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

add_executable(vc_smoothNormalsExample smoothNormalsExample.cpp)
target_link_libraries(vc_smoothNormalsExample ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

add_executable(vc_resliceAnalysis ResliceAnalysis.cpp)
target_link_libraries(vc_resliceAnalysis ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

if(ACVD_FOUND)
    add_executable(vc_ACVDExample ACVDExample.cpp)
    target_link_libraries(vc_ACVDExample ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})
endif(ACVD_FOUND)

#if(BULLET_FOUND)
#    add_executable(vc_clothExample clothExample.cpp)
#    target_link_libraries(vc_clothExample ${VC_LIBS} ${VC_REQ_LIBS})
#endif(BULLET_FOUND)

###############
#  texturing  #
###############

add_executable(vc_lscmExample lscmExample.cpp)
target_link_libraries(vc_lscmExample ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

###############
#  CNN Tools  #
###############

add_executable(vc_verifyStructureTensor VerifyStructureTensor.cpp)
target_link_libraries(vc_verifyStructureTensor ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_structureTensorClient StructureTensorClient.cpp)
target_link_libraries(vc_structureTensorClient ${VC_LIBS} ${VC_REQ_LIBS})

# Need exact version for outputting correctly-formatted HDF5 files for caffe to read in
find_package(HDF5 1.8.15 EXACT)
if(HDF5_FOUND)
    include_directories(${HDF5_INCLUDE_DIRS})
    add_executable(vc_makeNeuralNetTrainingData MakeNeuralNetTrainingData.cpp)
    target_link_libraries(vc_makeNeuralNetTrainingData ${HDF5_LIBRARIES} ${VC_LIBS} ${VC_REQ_LIBS})
endif(HDF5_FOUND)
