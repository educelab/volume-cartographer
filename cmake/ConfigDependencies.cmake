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

# Eigen (make it into a target)
find_package(Eigen3 REQUIRED)
add_library(eigen3 INTERFACE IMPORTED)
set_target_properties(eigen3
    PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIR}
)

# Rest
set(PCL_STATIC on)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)

if (VC_USE_ACVD)
    find_package(ACVD REQUIRED)
endif()
if (VC_USE_BULLET)
    find_package(Bullet REQUIRED)
endif()
if (VC_USE_LIBIGL)
    find_package(LIBIGL REQUIRED)
endif()
