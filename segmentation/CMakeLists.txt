project(libvc_segmentation VERSION ${VC_VERSION} LANGUAGES CXX)

set(srcs
    src/Common.cpp
    src/EnergyMetrics.cpp
    src/ForceChain.cpp
    src/FittedCurve.cpp
    src/FloodFill.cpp
    src/IntensityMap.cpp
    src/LocalResliceParticleSim.cpp
    src/OpticalFlowSegmentation.cpp
    src/Particle.cpp
    src/ParticleChain.cpp
    src/StructureTensorParticleSim.cpp
    src/ThinnedFloodFillSegmentation.cpp
    src/ComputeVolumetricMask.cpp
    src/CubicSplineMT.cpp
)

add_library(vc_segmentation ${srcs})
add_library(VC::segmentation ALIAS "vc_segmentation")
target_include_directories(vc_segmentation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_options(vc_segmentation
    PUBLIC
        -Wno-c++11-narrowing
)

include_directories(${GSL_INCLUDE_DIRS})
target_link_libraries(vc_segmentation
    PUBLIC
        VC::core
        Eigen3::Eigen
    PRIVATE
        opencv_highgui
        opencv_video
        ${VC_FS_LIB}
        GSL::gsl
)
target_compile_features(vc_segmentation PUBLIC cxx_std_17)
set_target_properties(vc_segmentation PROPERTIES
    VERSION "${PROJECT_VERSION}"
    EXPORT_NAME segmentation
)

if(VC_INSTALL_LIBS)
install(
    TARGETS vc_segmentation
    EXPORT ${targets_export_name}
    COMPONENT "Libraries"
    ARCHIVE DESTINATION "lib"
    LIBRARY DESTINATION "lib"
    RUNTIME DESTINATION "bin"
    INCLUDES DESTINATION "${include_install_dir}/segmentation"
)
install(
    DIRECTORY "${include_install_dir}/segmentation"
    DESTINATION "${include_install_dir}"
    COMPONENT "Libraries"
    FILES_MATCHING REGEX ".*\.(h|hpp)$"
)
endif()

### Testing ###
if(VC_BUILD_TESTS)
set(test_srcs
    test/CommonTest.cpp
    test/CubicSplineTest.cpp
    test/DerivativeTest.cpp
    test/EnergyMetricsTest.cpp
    test/FittedCurveTest.cpp
    test/IntensityMapTest.cpp
    test/LocalResliceParticleSimTest.cpp
)

# Add a test executable for each src
foreach(src ${test_srcs})
    get_filename_component(filename ${src} NAME_WE)
    set(testname vc_segmentation_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        VC::core
        VC::segmentation
        VC::testing
        gtest_main
        gmock_main
    )
    add_test(
        NAME ${testname}
        WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
        COMMAND ${testname}
    )
endforeach()
endif()
