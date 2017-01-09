project(libvc_segmentation VERSION 0.1 LANGUAGES CXX)
set(module "segmentation")
set(target "vc_${module}")

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
    VC::core
    opencv_highgui
)

add_library(${target} ${srcs})
add_library("${namespace}${module}" ALIAS "${target}")
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${include_install_dir}>
)
target_compile_options(${target}
	PUBLIC
		-Wno-c++11-narrowing
)
target_link_libraries(${target}
    PUBLIC
        ${deps}
)
set_target_properties(${target} PROPERTIES
    VERSION "${PROJECT_VERSION}"
    EXPORT_NAME "${module}"
)

install(
    TARGETS ${target}
    EXPORT ${targets_export_name}
    ARCHIVE DESTINATION "lib"
    LIBRARY DESTINATION "lib"
    RUNTIME DESTINATION "bin"
    INCLUDES DESTINATION "${include_install_dir}/${module}"
)
install(
    DIRECTORY "include/${module}"
    DESTINATION "${include_install_dir}"
    FILES_MATCHING REGEX ".*\.(h|hpp)$"
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
        VC::core
        VC::segmentation
        VC::testing
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
endif()
