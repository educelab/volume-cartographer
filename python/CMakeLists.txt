## Python wrapper utilities library ##

add_library(vc_python INTERFACE)
add_library(VC::python ALIAS vc_python)
target_include_directories(vc_python
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(vc_python INTERFACE cxx_std_17)
set_target_properties(vc_python PROPERTIES
    EXPORT_NAME python
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
install(CODE "execute_process(COMMAND ${PYTHON_EXECUTABLE} ${SETUP_PY} install --prefix ${CMAKE_INSTALL_PREFIX})")
endif()
