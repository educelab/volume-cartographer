project(libvc_app_support VERSION ${VC_VERSION} LANGUAGES CXX)

# Command line app support library
set(app_support_srcs src/GeneralOptions.cpp src/GetMemorySize.cpp)

add_library(app_support STATIC ${app_support_srcs})
add_library(VC::app_support ALIAS app_support)
target_include_directories(app_support PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include_app_support/>
)
target_link_libraries(app_support
    PUBLIC
        Boost::program_options
    INTERFACE
        indicators::indicators
)
target_compile_features(app_support PUBLIC cxx_std_17)

# Qt GUI app support library

# List of headers which need MOC
set(gui_support_hdrs
    include_gui_support/vc/gui_support/FetchSliceThread.hpp
    include_gui_support/vc/gui_support/ImageScrollArea.hpp
)
# List of source files
set(gui_support_srcs
    src/FetchSliceThread.cpp
    src/ImageScrollArea.cpp
)
# AUTOMOC doesn't work on libs so run MOC manually. Adds generated files to srcs
qt_wrap_cpp(gui_support_srcs ${gui_support_hdrs})

add_library(gui_support STATIC ${gui_support_srcs})
add_library(VC::gui_support ALIAS gui_support)
target_include_directories(gui_support PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include_gui_support/>
)
target_link_libraries(gui_support
    PUBLIC
        Qt6::Widgets
        opencv_core
        VC::core
    PRIVATE
        Qt6::Gui
)
target_compile_features(gui_support PUBLIC cxx_std_17)