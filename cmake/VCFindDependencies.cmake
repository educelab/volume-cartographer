########
# Core #
########

### Use VC Prebuilt Libs ###
# Built using vc-deps
option(VC_PREBUILT_LIBS "Link against prebuilt dependencies" off)
if (VC_PREBUILT_LIBS)
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/vc-deps/deps)
endif()

# For compiler sanitizers. Taken from:
# https://github.com/arsenm/sanitizers-cmake/blob/master/README.md
find_package(Sanitizers)

### Boost ###
set(VC_BOOST_COMPONENTS
    system
    filesystem
    program_options
    iostreams
)
find_package(Boost 1.58 REQUIRED COMPONENTS ${VC_BOOST_COMPONENTS})

### Qt5 ###
find_package(Qt5 5.7 QUIET REQUIRED COMPONENTS Widgets Gui Core)

### ITK ###
find_package(ITK 4.10 QUIET REQUIRED)
include(${ITK_USE_FILE})

### VTK ###
# VTK's config only supports minimum version up to the next major release
find_package(VTK 7 QUIET)
if(NOT VTK_FOUND)
    find_package(VTK 8 QUIET REQUIRED)
endif()
include(${VTK_USE_FILE})

# VTK does not mark its headers as system headers with -isystem, which makes
# warnings from those headers show up in builds. This marks them as "system"
# headers.
include_directories(SYSTEM ${VTK_INCLUDE_DIRS})

### Eigen ###
find_package(Eigen3 3.2 REQUIRED)

### OpenCV ###
find_package(OpenCV 3 REQUIRED)

### libtiff ###
find_package(TIFF 4.0 REQUIRED)

### spdlog ###
find_package(spdlog CONFIG REQUIRED)

############
# Optional #
############

### gtest ###
if(VC_BUILD_TESTS)
    include(FetchContent)

    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        release-1.8.1
        CMAKE_CACHE_ARGS
            -DINSTALL_GTEST:BOOL=OFF
    )

    FetchContent_GetProperties(googletest)
    if(NOT googletest_POPULATED)
        FetchContent_Populate(googletest)
        add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
    endif()
    set(INSTALL_GTEST OFF CACHE BOOL OFF FORCE)
endif()

# Python bindings
if(VC_BUILD_PYTHON_BINDINGS)
    find_package(pybind11 REQUIRED)
endif()

### OSX Code Signing ###
if(APPLE AND VC_BUILD_APPS)
    find_library(OSXSecurity Security)
endif()

# If this option is set, then use all optional dependencies
option(VC_USE_OPTIONAL "Enable all optional third-party libs" off)
if(VC_USE_OPTIONAL)
    message(STATUS "All optional third-party libraries enabled. Individual \
preferences will be ignored.")
endif()

### ACVD ###
# Currently required since VC-Texture needs it - SP
option(VC_USE_ACVD "Use ACVD library" on)
if (VC_USE_ACVD OR VC_USE_OPTIONAL)
    find_package(ACVD 1.0.1 REQUIRED)
endif()

### VCG ###
option(VC_USE_VCG "Use VCG library" off)
if(VC_USE_VCG OR VC_USE_OPTIONAL)
    find_package(VCG QUIET REQUIRED)
    add_library(vcglib INTERFACE IMPORTED)
    set_target_properties(vcglib PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${VCG_INCLUDE_DIRS}"
    )

    # Install a custom Find module
    list(APPEND VC_CUSTOM_MODULES "${CMAKE_SOURCE_DIR}/cmake/FindVCG.cmake")
endif()
