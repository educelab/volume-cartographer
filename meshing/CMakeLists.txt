cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(MESHING_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(MESHING_INCLUDE_DIR ${MESHING_INCLUDE_DIR} PARENT_SCOPE)

if(TARGET vc_meshing)
message("meshing is already defined")
else()
set(meshing_SOURCES orderedPCDMesher.h orderedPCDMesher.cpp
                    resamplePointCloud.h resamplePointCloud.cpp
                    greedyProjectionMeshing.h greedyProjectionMeshing.cpp
                    poissonReconstruction.h poissonReconstruction.cpp
                    itk2vtk.h itk2vtk.cpp
                    rayTrace.h rayTrace.cpp
                    scaleMesh.h scaleMesh.cpp
                    smoothNormals.h smoothNormals.cpp
                    deepCopy.cpp deepCopy.h)

#Conditional compiles
if(ACVD_FOUND)
     set(meshing_SOURCES ${meshing_SOURCES} ACVD.h ACVD.cpp)
endif(ACVD_FOUND)

if(BULLET_FOUND)
    set(meshing_SOURCES ${meshing_SOURCES} itk2bullet.h itk2bullet.cpp)
endif(BULLET_FOUND)

add_library(vc_meshing STATIC ${meshing_SOURCES})
endif()
