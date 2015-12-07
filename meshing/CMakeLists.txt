cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(MESHING_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(MESHING_INCLUDE_DIR ${MESHING_INCLUDE_DIR} PARENT_SCOPE)

if(TARGET vc_meshing)
message("meshing is already defined")
else()
add_library(vc_meshing STATIC
                    orderedPCDMesher.h orderedPCDMesher.cpp
                    resamplePointCloud.h resamplePointCloud.cpp
                    greedyProjectionMeshing.h greedyProjectionMeshing.cpp
                    poissonReconstruction.h poissonReconstruction.cpp
                    itk2vtk.h itk2vtk.cpp
                    rayTrace.h rayTrace.cpp
                    scaleMesh.h scaleMesh.cpp
                    smoothNormals.h smoothNormals.cpp
                    itk2bullet.h itk2bullet.cpp )
endif()
