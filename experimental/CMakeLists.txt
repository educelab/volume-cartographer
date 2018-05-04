project(libvc_experimental VERSION ${VC_VERSION} LANGUAGES C CXX)
set(module "experimental")
set(target "vc_${module}")


### Dependencies ###
find_package(Bullet QUIET REQUIRED)
add_library(bullet INTERFACE IMPORTED)
set_target_properties(bullet PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${BULLET_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${BULLET_LIBRARIES}"
)

### Setup sources and link libs ###
set(srcs
    src/ClothModelingUVMapping.cpp
    src/ITK2Bullet.cpp
)
set(public_deps
    VC::core
    bullet
    ${BULLET_LIBRARIES}
)
set(private_deps
    VC::meshing
)
set(defs
    VC_USE_BULLET
)

### Setup this library ###
add_library(${target} ${srcs})
add_library("${namespace}${module}" ALIAS "${target}")
target_compile_definitions(${target} PUBLIC ${defs})
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${target}
    PUBLIC ${public_deps}
    PRIVATE ${private_deps}
)
set_target_properties(${target} PROPERTIES
    VERSION "${PROJECT_VERSION}"
    EXPORT_NAME "${module}"
)

### Install lib ###
if(VC_INSTALL_LIBS)
install(
    TARGETS ${target}
    COMPONENT "Libraries"
    EXPORT ${targets_export_name}
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

# Set source files
set(test_srcs
    test/ClothTest.cpp
)

# Add a test executable for each src
foreach(src ${test_srcs})
    get_filename_component(filename ${src} NAME_WE)
    set(testname ${target}_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
    VC::experimental
    VC::testing
    Boost::unit_test_framework
    ${public_deps}
)
add_test(
    NAME ${testname}
    WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
    COMMAND ${testname}
)
endforeach()

set(VC_EXPERIMENTAL_TEST_RES
    test/res/clothUV_Arch_Unfurl.obj
    test/res/clothUV_Arch_Collide.obj
    test/res/clothUV_Arch_Final.obj
)

# Copy each test resource file
foreach(r ${VC_EXPERIMENTAL_TEST_RES})
    file(COPY ${r} DESTINATION ${EXECUTABLE_OUTPUT_PATH})
endforeach()

endif(VC_BUILD_TESTS)
