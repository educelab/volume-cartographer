# Where to look for deps
if (VC_PREBUILT_LIBS)
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/vc-deps/deps)
endif()

# Boost
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

# Qt stuff
find_package(Qt5 REQUIRED COMPONENTS Widgets Gui)

# ITK
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

# VTK
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
set(VTK_LIBRARIES_TMP ${VTK_LIBRARIES})

# Eigen (make it into a target)
find_package(Eigen3 REQUIRED)
add_library(eigen3 INTERFACE IMPORTED)
set_target_properties(eigen3 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
)

# Rest
set(PCL_STATIC on)
find_package(PCL 1.7 REQUIRED QUIET)

# XXX Put VTK libraries back because PCL silently overwrites them
set(VTK_LIBRARIES ${VTK_LIBRARIES_TMP})

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)

if (VC_USE_ACVD)
    find_package(ACVD REQUIRED)
    add_library(acvd INTERFACE IMPORTED)
    set_target_properties(acvd PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ACVD_INCLUDE_DIRS}"
        # Hack to get around the fact that ACVD_LIBRARIES doesn't contain the
        # needed VTK libs to actually compile it...
        INTERFACE_LINK_LIBRARIES "${ACVD_LIBRARIES};${VTK_LIBRARIES}"
    )
endif()
if (VC_USE_BULLET)
    find_package(Bullet REQUIRED)
    add_library(bullet INTERFACE IMPORTED)
    set_target_properties(bullet PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${BULLET_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${BULLET_LIBRARIES}"
    )
endif()
if (VC_USE_LIBIGL)
    find_package(LIBIGL REQUIRED)
    add_library(libigl INTERFACE IMPORTED)
    set_target_properties(libigl PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${LIBIGL_INCLUDE_DIRS}"
    )
endif()
