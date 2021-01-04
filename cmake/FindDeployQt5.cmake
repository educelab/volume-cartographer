include(FindPackageHandleStandardArgs)

if(NOT Qt5_FOUND)
    find_package(Qt5 COMPONENTS core)
endif()

if(Qt5Core_FOUND)
    get_target_property(_qmake_bin Qt5::qmake IMPORTED_LOCATION)
    get_filename_component(_qt_bin_dir "${_qmake_bin}" DIRECTORY)
    find_program(DeployQt5_EXECUTABLE NAMES windeployqt macdeployqt HINTS "${_qt_bin_dir}")
endif()

find_package_handle_standard_args(DeployQt5 DEFAULT_MSG DeployQt5_EXECUTABLE)

function(qt5_deploy_bundle TARGET)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(DeployFlags -no-strip)
    endif()
    message("Deploy flags: ${DeployFlags}")
    add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND "${DeployQt5_EXECUTABLE}" "$<TARGET_BUNDLE_DIR:${TARGET}>" "${DeployFlags}"
    )
endfunction()