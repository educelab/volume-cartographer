########
# Core #
########
### Use VC Prebuilt Libs ###
# Built using vc-deps
option(VC_PREBUILT_LIBS "Link against prebuilt dependencies" off)
if (VC_PREBUILT_LIBS)
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/vc-deps/deps)
endif()

### Filesystem ###
find_package(Filesystem)
if(Filesystem_FOUND)
    option(VC_USE_BOOSTFS "Use Boost as the filesystem library" OFF)
else()
    option(VC_USE_BOOSTFS "Use Boost as the filesystem library" ON)
endif()

if(VC_USE_BOOSTFS)
    add_compile_definitions(VC_USE_BOOSTFS)
    find_package(Boost 1.58 REQUIRED COMPONENTS system filesystem)
    set(VC_FS_LIB Boost::filesystem)
else()
    set(VC_FS_LIB std::filesystem)
endif()
message(STATUS "Using filesystem library: ${VC_FS_LIB}")
list(APPEND VC_CUSTOM_MODULES "${CMAKE_MODULE_PATH}/FindFilesystem.cmake")

### Qt5 ###
find_package(Qt5 5.9 QUIET REQUIRED COMPONENTS Widgets Gui Core Network)

### ITK ###
set(VC_ITK_COMPONENTS
    ITKCommon
    ITKMesh
    ITKQuadEdgeMesh
    ITKTransform
    ITKIOTransformBase
    ITKIOTransformInsightLegacy
)
find_package(ITK 4.10 COMPONENTS ${VC_ITK_COMPONENTS} QUIET REQUIRED)
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

### ACVD ###
include(BuildACVD)

### Eigen ###
find_package(Eigen3 3.3 REQUIRED)
if(CMAKE_GENERATOR MATCHES "Ninja|.*Makefiles.*" AND "${CMAKE_BUILD_TYPE}" MATCHES "^$|Debug")
    message(AUTHOR_WARNING "Configuring a Debug build. Eigen performance will be degraded. If you need debug symbols, \
    consider setting CMAKE_BUILD_TYPE to RelWithDebInfo. Otherwise, set to Release to maximize performance.")
endif()

### OpenCV ###
find_package(OpenCV 3 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 4 QUIET REQUIRED)
endif()

### libtiff ###
find_package(TIFF 4.0 REQUIRED)

### spdlog ###
find_package(spdlog 1.4.2 CONFIG REQUIRED)

### OpenABF ###
include(BuildOpenABF)

### Modern JSON ###
include(BuildJSON)

### bvh ###
include(Buildbvh)

### smeagol ###
include(BuildSmeagol)

### Boost and indicators (for app use only)
if(VC_BUILD_APPS OR VC_BUILD_UTILS)
    find_package(Boost 1.58 REQUIRED COMPONENTS system program_options)
    include(BuildIndicators)
endif()


############
# Optional #
############

### gtest ###
if(VC_BUILD_TESTS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        389cb68
        CMAKE_CACHE_ARGS
            -DINSTALL_GTEST:BOOL=OFF
    )

    FetchContent_GetProperties(googletest)
    if(NOT googletest_POPULATED)
        set(INSTALL_GTEST OFF CACHE BOOL OFF FORCE)
        FetchContent_Populate(googletest)
        add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()
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
