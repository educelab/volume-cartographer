include(FindPackageHandleStandardArgs)

if(NOT Qt6_FOUND)
    find_package(Qt6 COMPONENTS core)
endif()

if(Qt6Core_FOUND)
    get_target_property(_qmake_bin Qt6::qmake IMPORTED_LOCATION)
    get_filename_component(_qt_bin_dir "${_qmake_bin}" DIRECTORY)
    find_program(DeployQt6_EXECUTABLE NAMES windeployqt macdeployqt HINTS "${_qt_bin_dir}")
endif()

find_package_handle_standard_args(DeployQt6 DEFAULT_MSG DeployQt6_EXECUTABLE)

#function(qt6_deploy_bundle TARGET)
#    if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
#        set(DeployFlags -no-strip)
#    endif()
#    add_custom_command(
#        TARGET ${TARGET} POST_BUILD
#        COMMAND "${DeployQt6_EXECUTABLE}" "$<TARGET_BUNDLE_DIR:${TARGET}>" "${DeployFlags}"
#    )
#endfunction()