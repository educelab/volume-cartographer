cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(TEXTURING_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(TEXTURING_INCLUDE_DIR ${TEXTURING_INCLUDE_DIR} PARENT_SCOPE)

if(TARGET texturing)
    message("Texturing library is already defined")
else()
    set(vc_texturing_SRC compositeTexture.cpp compositeTexture.h
                         compositeTextureV2.cpp compositeTextureV2.h
                         simpleUV.h simpleUV.cpp
                         AngleBasedFlattening.h AngleBasedFlattening.cpp)

    if(VC_USE_BULLET)
        set( vc_texturing_SRC ${vc_texturing_SRC} clothModelingUV.h clothModelingUV.cpp)
    endif(VC_USE_BULLET)

    if(VC_USE_LIBIGL)
        set( vc_texturing_SRC ${vc_texturing_SRC} lscm.h lscm.cpp)
    endif(VC_USE_LIBIGL)

    add_library(vc_texturing STATIC ${vc_texturing_SRC})
endif()