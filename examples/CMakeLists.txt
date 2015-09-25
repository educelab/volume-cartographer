cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

enable_testing()

# volpkg
add_executable(vc_volpkg_example volpkg.cpp)
target_link_libraries(vc_volpkg_example ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_objExporterExample objExporterExample.cpp )
target_link_libraries(vc_objExporterExample ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_itk2vtkExample itk2vtkExample.cpp)
target_link_libraries(vc_itk2vtkExample ${VC_LIBS} ${VC_REQ_LIBS})

foreach(testSrc ${TEST_SRCS})

    get_filename_component(testName ${testSrc} NAME_WE)
    add_executable(${testName} ${testSrc})
    target_link_libraries(${testName} ${VC_LIBS} ${VC_REQ_LIBS})     #should pull Boost
    add_test(NAME ${testName} COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/testing/${testName})

endforeach(testSrc)
