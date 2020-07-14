project(libvc_app_support VERSION ${VC_VERSION} LANGUAGES CXX)

add_library(app_support INTERFACE)
add_library(VC::app_support ALIAS app_support)
target_include_directories(app_support INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(app_support INTERFACE indicators::indicators)