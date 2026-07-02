########
# Core #
########
### Use VC Prebuilt Libs ###
# Built using vc-deps
option(VC_PREBUILT_LIBS "Link against prebuilt dependencies" off)
if (VC_PREBUILT_LIBS)
    list(APPEND CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/vc-deps/deps)
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
    find_package(Boost 1.70 CONFIG REQUIRED COMPONENTS filesystem)
    set(VC_FS_LIB Boost::filesystem)
else()
    set(VC_FS_LIB std::filesystem)
endif()
message(STATUS "Using filesystem library: ${VC_FS_LIB}")
list(APPEND VC_CUSTOM_MODULES "${CMAKE_MODULE_PATH}/FindFilesystem.cmake")

### OpenMP ###
find_package(OpenMP)
cmake_dependent_option(VC_USE_OPENMP "Compile with OpenMP support" ON OpenMP_CXX_FOUND OFF)

### Qt6 ###
if((VC_BUILD_APPS OR VC_BUILD_UTILS) AND VC_BUILD_GUI)
    find_package(Qt6 6.5 QUIET REQUIRED COMPONENTS Widgets Gui Core Network)
    qt_standard_project_setup()
endif()

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
find_package(VTK 9 QUIET REQUIRED)

### ACVD ###
include(BuildACVD)

### Eigen ###
find_package(Eigen3 5 QUIET CONFIG)
if(NOT Eigen3_FOUND)
    find_package(Eigen3 3.3 REQUIRED CONFIG)
endif()
if(CMAKE_GENERATOR MATCHES "Ninja|.*Makefiles.*" AND "${CMAKE_BUILD_TYPE}" MATCHES "^$|Debug")
    message(AUTHOR_WARNING "Configuring a Debug build. Eigen performance will \
    be degraded. If you need debug symbols, consider setting CMAKE_BUILD_TYPE \
    to RelWithDebInfo. Otherwise, set to Release to maximize performance.")
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
find_package(nlohmann_json 3.9.1 CONFIG REQUIRED)

### bvh ###
include(Buildbvh)

### smgl ###
# PUBLIC dependency of vc_core; must be installed on the system, not fetched.
find_package(smgl 0.11.0 CONFIG REQUIRED)

### libcore ###
# PUBLIC dependency of vc_texturing; must be installed on the system.
find_package(EduceLabCore 0.3.0 CONFIG REQUIRED)

### Boost and indicators (for app use only)
if(VC_BUILD_APPS OR VC_BUILD_UTILS)
    find_package(Boost 1.70 CONFIG REQUIRED COMPONENTS program_options)
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
        GIT_TAG        v1.14.0
        EXCLUDE_FROM_ALL
    )
    set(INSTALL_GTEST OFF CACHE INTERNAL "")
    FetchContent_MakeAvailable(googletest)
endif()

# Python bindings
if(VC_BUILD_PYTHON_BINDINGS)
    find_package(pybind11 REQUIRED)
endif()

### macOS security framework ###
if(APPLE AND VC_BUILD_APPS AND VC_BUILD_GUI)
    find_library(OSXSecurity Security)
endif()
