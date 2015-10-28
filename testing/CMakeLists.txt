cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# tests below can be uncommented as test files are written
# tests split up below by folder withing v-c/testing/
# this cmake list should include all of the initial tests to be implemented

#-----------------------#
#    common Folder      #
#-----------------------#

add_executable(vc_test_OBJExporter common/ObjWriterTest.cpp)
target_link_libraries(vc_test_OBJExporter ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME OBJExporter WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_OBJExporter )

#add_executable(vc_test_ply2itk common/ply2itkTest.cpp)
#target_link_libraries(vc_test_ply2itk ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME ply2itk WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_ply2itk )

#-----------------------#
#    examples Folder    #
#-----------------------#

add_executable(vc_test_Sample examples/sampleTest.cpp)
target_link_libraries(vc_test_Sample ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME Sample WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_Sample )

#-----------------------#
#    meshing Folder     #
#-----------------------#

#add_executable(vc_test_greedyProjectionMeshing meshing/greedyProjectionMeshingTest.cpp)
#target_link_libraries(vc_test_greedyProjectionMeshing ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME greedyProjectionMeshing WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_greedyProjectionMeshing )

add_executable(vc_test_itk2vtk meshing/itk2vtkTest.cpp)
target_link_libraries(vc_test_itk2vtk ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME itk2vtk WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_itk2vtk )

#add_executable(vc_test_orderedPCDMesher meshing/orderedPCDMesherTest.cpp)
#target_link_libraries(vc_test_orderedPCDMesher ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME orderedPCDMesher WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_orderedPCDMesher )

#add_executable(vc_test_poissonReconstruction meshing/poissonReconstructionTest.cpp)
#target_link_libraries(vc_test_poissonReconstruction ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME poissonReconstruction WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_poissonReconstruction )

#add_executable(vc_test_rayTrace meshing/rayTraceTest.cpp)
#target_link_libraries(vc_test_rayTrace ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME rayTrace WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_rayTrace )

add_executable(vc_test_resamplePointCloud meshing/resamplePointCloudTest.cpp)
target_link_libraries(vc_test_resamplePointCloud ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME resamplePointCloud WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_resamplePointCloud )


#-----------------------#
#  segmentation Folder  #
#-----------------------#

##  localParticleSim ##

#add_executable(vc_test_chain segmentation/localResliceParticleSim/chainTest.cpp)
#target_link_libraries(vc_test_chain ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME chain WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_chain )

#add_executable(vc_test_field segmentation/localResliceParticleSim/fieldTest.cpp)
#target_link_libraries(vc_test_field ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME field WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_field )

#add_executable(vc_test_localResliceParticleSim segmentation/localResliceParticleSim/localResliceParticleSimTest.cpp)
#target_link_libraries(vc_test_localResliceParticleSim ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME localResliceParticleSim WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_localResliceParticleSim )

#add_executable(vc_test_particle segmentation/localResliceParticleSim/particleTest.cpp)
#target_link_libraries(vc_test_particle ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME particle WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_particle )

#add_executable(vc_test_slice segmentation/localResliceParticleSim/sliceTest.cpp)
#target_link_libraries(vc_test_slice ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME slice WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_slice )


##  structureTensorParticleSim ##

#add_executable(vc_test_tensor_chain segmentation/structureTensorParticleSim/tensorChainTest.cpp)
#target_link_libraries(vc_test_tensor_chain ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME tensor_chain WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_tensor_chain )

#add_executable(vc_test_tensor_field segmentation/structureTensorParticleSim/tensorFieldTest.cpp)
#target_link_libraries(vc_test_tensor_field ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME tensor_field WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_tensor_field )

#add_executable(vc_test_structureTensorParticleSim segmentation/structureTensorParticleSim/structureTensorParticleSimTest.cpp)
#target_link_libraries(vc_test_structureTensorParticleSim ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME structureTensorParticleSim WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_structureTensorParticleSim )

#add_executable(vc_test_tensor_particle segmentation/structureTensorParticleSim/tensorParticleTest.cpp)
#target_link_libraries(vc_test_tensor_particle ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME tensor_particle WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_tensor_particle )


#-----------------------#
#    texturing Folder   #
#-----------------------#

#add_executable(vc_test_layering texturing/layeringTest.cpp)
#target_link_libraries(vc_test_layering ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME layering WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_layering )

#add_executable(vc_test_main texturing/mainTest.cpp)
#target_link_libraries(vc_test_main ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME main WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_main )

#add_executable(vc_test_meshUtils texturing/meshUtilsTest.cpp)
#target_link_libraries(vc_test_meshUtils ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME meshUtils WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_meshUtils )

#add_executable(vc_test_UPointMapping texturing/UPointMappingTest.cpp)
#target_link_libraries(vc_test_UPointMapping ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME UPointMapping WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_UPointMapping )

#-----------------------#
#    volumepkg Folder   #
#-----------------------#

#add_executable(vc_test_volumepkg volumepkg/volumepkgTest.cpp)
#target_link_libraries(vc_test_volumepkg ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME volumepkg WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_volumepkg )

#add_executable(vc_test_volumepkgcfg volumepkg/volumepkgcfgTest.cpp)
#target_link_libraries(vc_test_volumepkgcfg ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME volumepkgcfg WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_volumepkgcfg )
