cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

set(SLS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(SLS_INCLUDE_DIR ${SLS_INCLUDE_DIR} PARENT_SCOPE)

if(TARGET vc_SparseLinearSolver)
    message("SparseLinearSolver library is already defined")
else()
    add_library( vc_SparseLinearSolver STATIC 
                    linear_solver.h linear_solver.cc
                    eigenvalues.h eigenvalues.cc
                    svd.h svd.cc )
endif()