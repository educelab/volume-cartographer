cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(VC_COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(VC_COMMON_INCLUDE_DIR ${VC_COMMON_INCLUDE_DIR} PARENT_SCOPE)

if(TARGET vc_common)
message("vc_common is already defined")
else()

set(COMMON_MetaHeaders
        vc_defines.h
        vc_datatypes.h
        shapes.h
        scales/scales.h )

set(COMMON_DatatypeSources
        datatypes/LRUCache.h datatypes/UVMap.h
        datatypes/Metadata.h datatypes/Metadata.cpp
        datatypes/Texture.h datatypes/Texture.cpp
        datatypes/Volume.h datatypes/Volume.cpp
        datatypes/Slice.h datatypes/Slice.cpp
        datatypes/PerPixelMap.h datatypes/PerPixelMap.cpp
        datatypes/Rendering.h datatypes/Rendering.cpp
        datatypes/HalfEdgeMesh.h datatypes/HalfEdgeMesh.cpp )

set(COMMON_ScaleSources
        scales/scale_pi.h
        scales/scale_micro.h
        scales/scale_small.h )

set(COMMON_IOSources
        io/objWriter.cpp io/objWriter.h
        io/plyWriter.cpp io/plyWriter.h
        io/ply2itk.h io/ply2itk.cpp )

set(COMMON_ShapeSources
        shapes/ShapePrimitive.h shapes/ShapePrimitive.cpp
        shapes/Plane.h
        shapes/Arch.h
        shapes/Sphere.h shapes/Sphere.cpp
        shapes/Cube.h shapes/Cone.h )

set(COMMON_UtilitiesSources
        getMemorySize.h getMemorySize.cpp
        meshMath.h meshMath.cpp )

add_library(vc_common STATIC
        ${COMMON_MetaHeaders}
        ${COMMON_DatatypeSources}
        ${COMMON_ScaleSources}
        ${COMMON_IOSources}
        ${COMMON_ShapeSources}
        ${COMMON_UtilitiesSources} )

endif()
