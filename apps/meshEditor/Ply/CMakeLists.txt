project( PLY )

# Check CMake version
cmake_minimum_required( VERSION 2.6.2 )
if( "${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" VERSION_GREATER 2.6 )
  if( "${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" VERSION_GREATER 2.8.3 )
    cmake_policy( VERSION 2.8.4 )
  else()
    cmake_policy( VERSION 2.6 )
  endif()
endif()

# Preprocessor directives
add_definitions( "/D_CRT_SECURE_NO_WARNINGS" )

set( HDRS base.h header.h object.h unknown.h io.h example.h )
set( SRCS base.cpp header.cpp object.cpp unknown.cpp io.cpp example.cpp )

# Set include directories
include_directories( BEFORE ./ )

add_executable( ply_example ${HDRS} ${SRCS} )
set_target_properties( ply_example PROPERTIES DEBUG_POSTFIX d )

install( TARGETS ply_example DESTINATION bin )