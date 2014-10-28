cmake_minimum_required(VERSION 2.8)

project(Utils)

find_package(OpenCV REQUIRED)

find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

add_executable(Histogram Histogram.cxx)
target_link_libraries(Histogram ${ITK_LIBRARIES})

add_executable(automask automask.cpp)
target_link_libraries(automask ${OpenCV_LIBS})
