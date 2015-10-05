cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_library(localResliceParticleSim OBJECT localResliceParticleSim.h localResliceParticleSim.cpp field.h field.cpp chain.h chain.cpp Particle.h Particle.cpp slice.h slice.cpp NormalizedIntensityMap.cpp NormalizedIntensityMap.h)