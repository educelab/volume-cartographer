cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(MESHING_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(MESHING_INCLUDE_DIR ${MESHING_INCLUDE_DIR} PARENT_SCOPE)

if(TARGET vc_meshing)
message("meshing is already defined")
else()
set(meshing_SOURCES orderedPCDMesher.h orderedPCDMesher.cpp
                    resamplePointCloud.h resamplePointCloud.cpp
                    greedyProjectionMeshing.h greedyProjectionMeshing.cpp
                    itk2vtk.h itk2vtk.cpp
                    rayTrace.h rayTrace.cpp
                    scaleMesh.h scaleMesh.cpp
                    smoothNormals.h smoothNormals.cpp
                    deepCopy.cpp deepCopy.h)

#Conditional compiles
if(VC_USE_ACVD)
     set(meshing_SOURCES ${meshing_SOURCES} ACVD.h ACVD.cpp)
endif(VC_USE_ACVD)

if(VC_USE_BULLET)
     set( meshing_SOURCES ${meshing_SOURCES} itk2bullet.h itk2bullet.cpp)
endif(VC_USE_BULLET)

add_library(vc_meshing STATIC ${meshing_SOURCES})
endif()
