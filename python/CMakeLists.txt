## Python wrapper utilities library ##
set(module "python")
set(target "vc_${module}")

add_library(${target} INTERFACE)
add_library("${namespace}${module}" ALIAS "${target}")
target_include_directories(${target}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
set_target_properties(${target} PROPERTIES
    EXPORT_NAME "${module}"
)

## PyPI Packaging ##
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/volcart/")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/volcart/__init__.py" "")

set(SETUP_PY_IN "${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in")
set(SETUP_PY    "${CMAKE_CURRENT_BINARY_DIR}/setup.py")
set(INIT_PY     "${CMAKE_CURRENT_BINARY_DIR}/volcart/__init__.py")
set(OUTPUT      "${CMAKE_CURRENT_BINARY_DIR}/build/timestamp")

configure_file(${SETUP_PY_IN} ${SETUP_PY})

add_custom_command(OUTPUT ${OUTPUT}
        COMMAND ${PYTHON_EXECUTABLE} ${SETUP_PY} build
        COMMAND ${CMAKE_COMMAND} -E touch ${OUTPUT}
        DEPENDS ${INIT_PY})

add_custom_target(build_py ALL DEPENDS ${OUTPUT} ${PYBIND_TARGETS})

if(VC_INSTALL_PYTHON_BINDINGS)
install(CODE "execute_process(COMMAND ${PYTHON_EXECUTABLE} ${SETUP_PY} install)")
endif()
