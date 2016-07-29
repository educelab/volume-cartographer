# Where to look for deps
if(USE_PREBUILT_LIBS)
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/vc-deps/deps)
endif()

# Find external libs and import their interfaces as targets
set(VC_BOOST_COMPONENTS system filesystem program_options iostreams test)
find_package(Boost REQUIRED COMPONENTS ${VC_BOOST_COMPONENTS})
if(USE_PREBUILT_LIBS)
    set(Boost_USE_STATIC_LIBS on)
endif()

# Qt stuff
set(CMAKE_AUTOMOC on)
find_package(Qt5 REQUIRED COMPONENTS Widets Gui)

# ITK
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

# VTK
find_package(VTK 6.1 REQUIRED)
include(${VTK_USE_FILE})

# PCL
find_package(PCL 1.7 REQUIRED)

find_package(OpenCV REQUIRED)

find_package(OpenGL REQUIRED)

if(LINK_ACVD)
    find_package(ACVD)
endif()
if(LINK_Bullet)
    find_package(Bullet)
endif()
