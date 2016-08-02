cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# tests below can be uncommented as test files are written
# tests split up below by folder withing v-c/testing/
# this cmake list should include all of the initial tests to be implemented


# executable location:
# currently, the tests are being built and placed in build/bin via the ${CMAKE_BINARY_DIR}/bin variable.
# In order to change this so that each of the executables is placed in a separate folder under testing,
# e.g., build/testing/meshing, the ${CMAKE_BINARY_DIR}/bin variable should be changed to
# ${CMAKE_CURRENT_BINARY_DIR} in the add_test(). If there are test_data files being copied into the
# executable location, that binary_dir variable will also need to be updated to reflect the change and
# ensure the project builds correctly for the tests.

set(TESTING_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(TESTING_INCLUDE_DIR ${TESTING_INCLUDE_DIR} PARENT_SCOPE)
include_directories(${TESTING_INCLUDE_DIR})

#-------------------------#
#    Global Test Data     #
#-------------------------#
# Copy the test volumepkg
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/volpkg/Testing.volpkg DESTINATION ${CMAKE_BINARY_DIR}/bin)

#-----------------------#
#    common Folder      #
#-----------------------#

# Copy the .ply file to the current bin
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/common/OBJWriterPlaneData.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
add_executable(vc_test_OBJExporter common/ObjWriterTest.cpp parsingHelpers.cpp)
target_link_libraries(vc_test_OBJExporter ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME OBJExporter WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_OBJExporter )

# Copy the .ply file to the current bin
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/common/output.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
add_executable(vc_test_plyWriter common/plyWriterTest.cpp parsingHelpers.cpp)
target_link_libraries(vc_test_plyWriter ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME plyWriter WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_plyWriter )

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/common/gradient.tif DESTINATION ${CMAKE_BINARY_DIR}/bin)
add_executable(vc_test_Texture common/TextureTest.cpp)
target_link_libraries(vc_test_Texture ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME Texture WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_Texture )

add_executable(vc_test_uvMap common/UVMapTest.cpp)
target_link_libraries(vc_test_uvMap ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME uvMap WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_uvMap )

add_executable(vc_test_LRUCache common/LRUCacheTest.cpp)
target_link_libraries(vc_test_LRUCache ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME LRUCache WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_LRUCache )

#-----------------------#
#    examples Folder    #
#-----------------------#

add_executable(vc_test_Sample examples/sampleTest.cpp)
target_link_libraries(vc_test_Sample ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME Sample WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_Sample )

#-----------------------#
#    meshing Folder     #
#-----------------------#

#copy vtk and itk test case files to build dir
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ITKPlaneMeshConvertedToVTK.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ITKCubeMeshConvertedToVTK.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ITKArchMeshConvertedToVTK.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ITKSphereMeshConvertedToVTK.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ITKConeMeshConvertedToVTK.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/VTKPlaneMeshConvertedToITK.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/VTKCubeMeshConvertedToITK.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/VTKArchMeshConvertedToITK.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/VTKSphereMeshConvertedToITK.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/VTKConeMeshConvertedToITK.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
add_executable(vc_test_itk2vtk meshing/itk2vtkTest.cpp parsingHelpers.cpp)
target_link_libraries(vc_test_itk2vtk ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME itk2vtk WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_itk2vtk )

# Copy the test files created by orderedPCDMesherExample.cpp
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/PlaneOrderedPCDMesher.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/CubeOrderedPCDMesher.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ArchOrderedPCDMesher.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/SphereOrderedPCDMesher.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ConeOrderedPCDMesher.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
add_executable(vc_test_orderedPCDMesher meshing/orderedPCDMesherTest.cpp parsingHelpers.cpp)
target_link_libraries(vc_test_orderedPCDMesher ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME orderedPCDMesher WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_orderedPCDMesher )

# Copy the test files created by rayTraceExample.cpp
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/SavedPlaneRayTraceData.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/SavedCubeRayTraceData.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/SavedArchRayTraceData.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/SavedConeRayTraceData.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/SavedSphereRayTraceData.ply DESTINATION ${CMAKE_BINARY_DIR}/bin)
add_executable(vc_test_rayTrace meshing/rayTraceTest.cpp parsingHelpers.cpp)
target_link_libraries(vc_test_rayTrace ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME rayTrace WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_rayTrace )

# Copy the test files created by resamplePointCloudExample.cpp to the build dir
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/PlaneResamplePointCloudExample.pcd DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/CubeResamplePointCloudExample.pcd DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ArchResamplePointCloudExample.pcd DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/SphereResamplePointCloudExample.pcd DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ConeResamplePointCloudExample.pcd DESTINATION ${CMAKE_BINARY_DIR}/bin)
add_executable(vc_test_resamplePointCloud meshing/resamplePointCloudTest.cpp)
target_link_libraries(vc_test_resamplePointCloud ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME resamplePointCloud WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_resamplePointCloud )

# Copy the 'smoothed' test files created by smoothNormalsExample.cpp
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/PlaneWithSmoothedNormals.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/CubeWithSmoothedNormals.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ArchWithSmoothedNormals.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/SphereWithSmoothedNormals.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ConeWithSmoothedNormals.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
add_executable(vc_test_smoothNormals meshing/smoothNormalsTest.cpp parsingHelpers.cpp)
target_link_libraries(vc_test_smoothNormals ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME smoothNormals WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_smoothNormals )

# Copy the scaledMesh saved test files created by scaleMeshExample.cpp
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ScaledPlaneMesh.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ScaledCubeMesh.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ScaledArchMesh.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ScaledSphereMesh.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ScaledConeMesh.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
add_executable(vc_test_scaleMesh meshing/scaleMeshTest.cpp parsingHelpers.cpp)
target_link_libraries(vc_test_scaleMesh ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME scaleMesh WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_scaleMesh )

#Copy the resampled test files created by orderedResamplingExample.cpp
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/PlaneOrderedResampling.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/meshing/ArchOrderedResampling.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
add_executable(vc_test_orderedResampling meshing/orderedResamplingTest.cpp parsingHelpers.cpp)
target_link_libraries(vc_test_orderedResampling ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME orderedResampling WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_orderedResampling )

# Calculate Normals
add_executable(vc_test_CalculateNormals meshing/CalculateNormalsTest.cpp)
target_link_libraries(vc_test_CalculateNormals ${VC_LIBS} vc_meshing ${VC_REQ_LIBS} ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME CalculateNormals WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_CalculateNormals )

#-----------------------#
#  segmentation Folder  #
#-----------------------#

##  localParticleSim ##
file(COPY
        ${CMAKE_BINARY_DIR}/bin/Testing.volpkg/paths/starting-path/cloud.pcd
    DESTINATION
        ${CMAKE_BINARY_DIR}/bin/Testing.volpkg/paths/lrps-test-results)
add_executable(vc_test_localResliceParticleSim
    segmentation/localResliceParticleSim/localResliceParticleSimTest.cpp)
target_link_libraries(vc_test_localResliceParticleSim
    ${VC_LIBS}
    ${VC_REQ_LIBS}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME localResliceParticleSim
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    COMMAND vc_test_localResliceParticleSim)

add_executable(vc_test_localResliceParticleSim_common
    segmentation/localResliceParticleSim/commonTest.cpp)
target_link_libraries(vc_test_localResliceParticleSim_common
    ${VC_LIBS}
    ${VC_REQ_LIBS}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME lrps_common
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    COMMAND vc_test_localResliceParticleSim_common)

add_executable(vc_test_localResliceParticleSim_cubicspline
    segmentation/localResliceParticleSim/cubicSplineTest.cpp)
target_link_libraries(vc_test_localResliceParticleSim_cubicspline
    ${VC_LIBS}
    ${VC_REQ_LIBS}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME lrps_spline
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    COMMAND vc_test_localResliceParticleSim_cubicspline)

add_executable(vc_test_localResliceParticleSim_fittedcurve
    segmentation/localResliceParticleSim/fittedCurveTest.cpp)
target_link_libraries(vc_test_localResliceParticleSim_fittedcurve
    ${VC_LIBS}
    ${VC_REQ_LIBS}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME lrps_fittedcurve
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    COMMAND vc_test_localResliceParticleSim_fittedcurve)

add_executable(vc_test_localResliceParticleSim_derivative
    segmentation/localResliceParticleSim/derivativeTest.cpp)
target_link_libraries(vc_test_localResliceParticleSim_derivative
    ${VC_LIBS}
    ${VC_REQ_LIBS}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME lrps_derivative
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    COMMAND vc_test_localResliceParticleSim_derivative)

add_executable(vc_test_localResliceParticleSim_intensitymap
    segmentation/localResliceParticleSim/intensitymapTest.cpp)
target_link_libraries(vc_test_localResliceParticleSim_intensitymap
    ${VC_LIBS}
    ${VC_REQ_LIBS}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME lrps_intensitymap
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    COMMAND vc_test_localResliceParticleSim_intensitymap)

add_executable(vc_test_localResliceParticleSim_energymetrics
    segmentation/localResliceParticleSim/energyMetricsTest.cpp)
target_link_libraries(vc_test_localResliceParticleSim_energymetrics
    ${VC_LIBS}
    ${VC_REQ_LIBS}
    ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME lrps_energymetrics
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    COMMAND vc_test_localResliceParticleSim_energymetrics)

#add_executable(vc_test_particle segmentation/localResliceParticleSim/particleTest.cpp)
#target_link_libraries(vc_test_particle ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
#add_test(NAME particle WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_particle )
#
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

# Cloth Modeling UV
if(VC_USE_BULLET)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/texturing/clothUV_Arch_Unfurl.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/texturing/clothUV_Arch_Collide.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/texturing/clothUV_Arch_Final.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
    add_executable(vc_test_clothUV texturing/clothTest.cpp parsingHelpers.cpp)
    target_link_libraries(vc_test_clothUV ${VC_LIBS} vc_meshing ${VC_REQ_LIBS} ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} )
    add_test(NAME ClothModelingUVMapping WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_clothUV )
endif(VC_USE_BULLET)

# Angle-based Flattening (ABF)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/texturing/abf_Plane.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/texturing/abf_Plane_LSCMOnly.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/texturing/abf_Arch.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/texturing/abf_Arch_LSCMOnly.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
add_executable(vc_test_abf texturing/abfTest.cpp parsingHelpers.cpp)
target_link_libraries(vc_test_abf ${VC_LIBS} vc_meshing ${VC_REQ_LIBS} vc_SparseLinearSolver ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} )
add_test(NAME AngleBasedFlattening WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_abf )

# Least Squares Conformal Maps (LSCM)
if(VC_USE_LIBIGL)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/texturing/lscm_Plane.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data/texturing/lscm_Arch.obj DESTINATION ${CMAKE_BINARY_DIR}/bin)
    add_executable(vc_test_lscm texturing/lscmTest.cpp parsingHelpers.cpp)
    target_link_libraries(vc_test_lscm ${VC_LIBS} vc_meshing ${VC_REQ_LIBS} ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} )
    add_test(NAME LeastSquaresConformalMapping WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_lscm )
endif(VC_USE_LIBIGL)

add_executable(vc_test_simpleUV texturing/simpleUVTest.cpp)
target_link_libraries(vc_test_simpleUV ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY} )
add_test(NAME simpleUV WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_simpleUV )

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



