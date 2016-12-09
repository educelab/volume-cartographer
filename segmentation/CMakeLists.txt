project(libvc_segmentation VERSION 0.1 LANGUAGES CXX)

set(component segmentation)
set(target "vc_${component}")

set(srcs
    src/chain.cpp
    src/common.cpp
    src/energymetrics.cpp
    src/fittedcurve.cpp
    src/intensitymap.cpp
    src/localResliceParticleSim.cpp
    src/particle.cpp
    src/structureTensorParticleSim.cpp
)
set(deps
    vc_core
)

add_library(${target} ${srcs})
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_options(${target}
	PUBLIC
		-Wno-c++11-narrowing
)
target_link_libraries(${target}
    PUBLIC
        ${deps}
)

set(export_config SegmentationConfig)
install(TARGETS ${target}
    EXPORT ${export_config}
    ARCHIVE DESTINATION lib COMPONENT ${component}
    LIBRARY DESTINATION lib COMPONENT ${component}
    RUNTIME DESTINATION bin
)
install(
    DIRECTORY include/
    DESTINATION ${VC_INSTALL_INCLUDE_PREFIX}
    COMPONENT ${component}
)
install(
    EXPORT ${export_config}
    DESTINATION share/${component}/cmake
)
export(
    TARGETS ${target}
    FILE ${export_config}.cmake
)

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
        vc_core
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
    DESTINATION
        ${EXECUTABLE_OUTPUT_PATH}/Testing.volpkg/paths/lrps-test-results
)
endif(VC_BUILD_TESTS)
