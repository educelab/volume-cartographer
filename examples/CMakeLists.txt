cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# volpkg
add_executable(vc_volpkg_example volpkg.cpp)
target_link_libraries(vc_volpkg_example ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_objExporterExample objExporterExample.cpp)
target_link_libraries(vc_objExporterExample ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_itk2vtkExample itk2vtkExample.cpp)
target_link_libraries(vc_itk2vtkExample ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_uvMapExample uvMapExample.cpp)
target_link_libraries(vc_uvMapExample ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_plyExporterExample plyExporterExample.cpp)
target_link_libraries(vc_plyExporterExample ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_verifyStructureTensor VerifyStructureTensor.cpp)
target_link_libraries(vc_verifyStructureTensor  ${VC_LIBS} ${VC_REQ_LIBS})
