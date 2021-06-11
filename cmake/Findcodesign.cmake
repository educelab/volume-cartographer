include(FindPackageHandleStandardArgs)

find_program(CodeSign_EXECUTABLE codesign)

find_package_handle_standard_args(CodeSign DEFAULT_MSG CodeSign_EXECUTABLE)

set(CodeSign_EXECUTABLE_NS "")

function(codesign_executable TARGET CERT_ID)
    if(DEFINED ARGV3)
        set(APP_ID ${ARGV3})
    else()
        set(APP_ID $<TARGET_FILE_BASE_NAME:${TARGET}>)
    endif()
    if(CodeSign_EXECUTABLE_NS)
        set(APP_ID ${CodeSign_EXECUTABLE_NS}.${APP_ID})
    endif()

    add_custom_command(
        TARGET ${TARGET} POST_BUILD
        COMMAND "${CodeSign_EXECUTABLE}" --deep --force --sign "${CERT_ID}" -i "${APP_ID}" "$<TARGET_FILE:${TARGET}>"
    )
endfunction()

function(codesign_bundle TARGET CERT_ID)
    add_custom_command(
            TARGET ${TARGET} POST_BUILD
            COMMAND "${CodeSign_EXECUTABLE}" --deep --force --sign "${CERT_ID}" "$<TARGET_BUNDLE_DIR:${TARGET}>"
    )
endfunction()