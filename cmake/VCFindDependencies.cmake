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
    unit_test_framework
)
find_package(Boost REQUIRED COMPONENTS ${VC_BOOST_COMPONENTS})
if (VC_PREBUILT_LIBS)
    set(Boost_USE_STATIC_LIBS on)
endif()

### Qt5 ###
find_package(Qt5 QUIET REQUIRED COMPONENTS Widgets Gui Core)

### ITK ###
find_package(ITK 4.10 QUIET REQUIRED)
include(${ITK_USE_FILE})

### VTK ###
find_package(VTK QUIET REQUIRED)
include(${VTK_USE_FILE})

# VTK does not mark its headers as system headers with -isystem, which makes
# warnings from those headers show up in builds. This marks them as "system"
# headers.
include_directories(SYSTEM ${VTK_INCLUDE_DIRS})

### Eigen ###
# XXX libigl requires Eigen 3.2.x, which doesn't support namespaced targets and
# transitively-included dependencies, so make it into a target
find_package(Eigen3 QUIET REQUIRED)
add_library(eigen3 INTERFACE IMPORTED)
set_target_properties(eigen3 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIR}
)

### OpenCV ###
find_package(OpenCV 3 REQUIRED)

############
# Optional #
############

# If this option is set, then use all optional dependencies
option(VC_USE_ALL "Use all optional third-party libs" off)

# Individual options for third-party dependencies
option(VC_USE_ACVD "Use ACVD library" on)
option(VC_USE_BULLET "Use Bullet Physics library" off)
option(VC_USE_LIBIGL "Use libigl" off)
option(VC_USE_VCG "Use VCG library" off)

if (VC_USE_ALL)
    set(VC_USE_ACVD on)
    set(VC_USE_BULLET on)
    set(VC_USE_LIBIGL on)
    set(VC_USE_VCG on)
endif()

### OSX Code Signing ###
if(APPLE AND VC_BUILD_APPS)
    find_library(OSXSecurity Security)
endif()

### ACVD ###
# Currently required since VC-Texture needs it - SP
if (VC_USE_ACVD)
    find_package(ACVD QUIET REQUIRED)
    add_library(acvd INTERFACE IMPORTED)
    set_target_properties(acvd PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ACVD_INCLUDE_DIRS}"
        # Hack to get around the fact that ACVD_LIBRARIES doesn't contain the
        # needed VTK libs to actually compile it...
        INTERFACE_LINK_LIBRARIES "${ACVD_LIBRARIES};${VTK_LIBRARIES}"
    )
endif()

### Bullet Physics ###
if (VC_USE_BULLET)
    find_package(Bullet QUIET REQUIRED)
    add_library(bullet INTERFACE IMPORTED)
    set_target_properties(bullet PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${BULLET_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${BULLET_LIBRARIES}"
    )
endif()

### libigl ###
if (VC_USE_LIBIGL)
    find_package(LIBIGL QUIET REQUIRED)
    add_library(libigl INTERFACE IMPORTED)
    set_target_properties(libigl PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${LIBIGL_INCLUDE_DIRS}"
    )
endif()

### VCG ###
if(VC_USE_VCG)
    find_package(VCG QUIET REQUIRED)
    add_library(vcglib INTERFACE IMPORTED)
    set_target_properties(vcglib PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${VCG_INCLUDE_DIRS}"
    )
endif()
