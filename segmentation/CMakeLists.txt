project(libvc_segmentation VERSION ${VC_VERSION} LANGUAGES CXX)
set(module "segmentation")
set(target "vc_${module}")

set(srcs
    src/Common.cpp
    src/EnergyMetrics.cpp
    src/ForceChain.cpp
    src/FittedCurve.cpp
    src/IntensityMap.cpp
    src/LocalResliceParticleSim.cpp
    src/Particle.cpp
    src/ParticleChain.cpp
    src/StructureTensorParticleSim.cpp
    src/ThinnedFloodFillSegmentation.cpp
)
set(deps
    VC::core
    opencv_highgui
    Eigen3::Eigen
)

add_library(${target} ${srcs})
add_library("${namespace}${module}" ALIAS "${target}")
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
set_target_properties(${target} PROPERTIES
    VERSION "${PROJECT_VERSION}"
    EXPORT_NAME "${module}"
)

if(VC_INSTALL_LIBS)
install(
    TARGETS ${target}
    EXPORT ${targets_export_name}
    COMPONENT "Libraries"
    ARCHIVE DESTINATION "lib"
    LIBRARY DESTINATION "lib"
    RUNTIME DESTINATION "bin"
    INCLUDES DESTINATION "${include_install_dir}/${module}"
)
install(
    DIRECTORY "${include_install_dir}/${module}"
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
