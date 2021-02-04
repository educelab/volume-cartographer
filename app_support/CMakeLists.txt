project(libvc_app_support VERSION ${VC_VERSION} LANGUAGES CXX)

set(srcs src/GeneralOptions.cpp src/GetMemorySize.cpp)

add_library(app_support ${srcs})
add_library(VC::app_support ALIAS app_support)
target_include_directories(app_support PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(app_support
    PUBLIC
        Boost::program_options
    INTERFACE
        indicators::indicators
)