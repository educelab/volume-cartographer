cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_library(localResliceParticleSim OBJECT localResliceParticleSim.h localResliceParticleSim.cpp chain.h chain.cpp particle.h particle.cpp NormalizedIntensityMap.cpp NormalizedIntensityMap.h chainmesh.cpp chainmesh.h common.h fittedcurve.h)
