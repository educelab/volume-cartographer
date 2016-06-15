cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(TEXTURING_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(TEXTURING_INCLUDE_DIR ${TEXTURING_INCLUDE_DIR} PARENT_SCOPE)

if(TARGET texturing)
    message("Texturing library is already defined")
else()
    add_library(vc_texturing STATIC compositeTexture.cpp compositeTexture.h
                                    compositeTextureV2.cpp compositeTextureV2.h
                                    clothModelingUV.cpp clothModelingUV.h
                                    lscm.cpp lscm.h
                                    simpleUV.h simpleUV.cpp
                                    abf.h abf.cpp )
endif()