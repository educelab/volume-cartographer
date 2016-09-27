project(libvc_segmentation VERSION 0.1 LANGUAGES CXX)

set(VC_SEGMENTATION_SRCS
    src/chain.cpp
    src/common.cpp
    src/energymetrics.cpp
    src/fittedcurve.cpp
    src/intensitymap.cpp
    src/localResliceParticleSim.cpp
    src/particle.cpp
    src/structureTensorParticleSim.cpp
)
set(VC_SEGMENTATION_DEPENDENCIES
    vc_common
    vc_volumepkg
)

add_library(vc_segmentation ${VC_SEGMENTATION_SRCS})
target_include_directories(vc_segmentation
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
# Get rid of annoying warnings
target_compile_options(vc_segmentation
    PRIVATE
        -Wno-c++11-narrowing
)
target_link_libraries(vc_segmentation
    PUBLIC
        vc_common
        vc_volumepkg
)

install(TARGETS vc_segmentation
    ARCHIVE DESTINATION lib COMPONENT Libraries
    LIBRARY DESTINATION lib COMPONENT Libraries
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX} COMPONENT Includes)
export(TARGETS vc_segmentation FILE VCSegmentation.cmake)

### Testing ###
if(VC_BUILD_TESTS)

# Set source files
set(VC_SEGMENTATION_TEST_SRCS
    test/commonTest.cpp
    test/cubicSplineTest.cpp
    test/derivativeTest.cpp
    test/energyMetricsTest.cpp
    test/fittedCurveTest.cpp
    test/intensitymapTest.cpp
    test/localResliceParticleSimTest.cpp
)

# Add a test executable for each src
foreach(src ${VC_SEGMENTATION_TEST_SRCS})
    get_filename_component(filename ${src} NAME_WE)
    set(testname vc_segmentation_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        vc_common
        vc_segmentation
        vc_testing
        Boost::unit_test_framework
    )
    add_test(
        NAME ${testname}
        WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
        COMMAND ${testname}
    )
endforeach()

# Copy test resource files
file(
    COPY
        ${EXECUTABLE_OUTPUT_PATH}/Testing.volpkg/paths/starting-path/pointset.vcps
        ${EXECUTABLE_OUTPUT_PATH}/Testing.volpkg/paths/starting-path/cloud.pcd
        DESTINATION
        ${EXECUTABLE_OUTPUT_PATH}/Testing.volpkg/paths/lrps-test-results
)
endif(VC_BUILD_TESTS)