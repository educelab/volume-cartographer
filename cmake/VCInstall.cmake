# Mostly taken from github:forexample/package-example

if(VC_INSTALL_LIBS)
# Generated files directory (usually build/bin/generated)
set(gen_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Generated CMake project and version configuration files
set(version_config "${gen_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${gen_dir}/${PROJECT_NAME}Config.cmake")

# Configures VCConfigVersion.cmake
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${version_config}" COMPATIBILITY SameMajorVersion
)

# Configures VCConfig.cmake
set(config_in "cmake/Config.cmake.in")
configure_package_config_file(
    "${config_in}"
    "${project_config}"
    INSTALL_DESTINATION "${config_install_dir}"
)

# Install VC project config + version config
install(
    FILES "${project_config}" "${version_config}"
    DESTINATION "${config_install_dir}"
    COMPONENT Libraries
)

# Install exported targets file
install(
    EXPORT "${targets_export_name}"
    NAMESPACE "${namespace}"
    DESTINATION "${config_install_dir}"
    COMPONENT Libraries
)
endif()

# What components to install
unset(install_components)
if(VC_INSTALL_APPS)
    set(install_components ${install_components} Programs)
endif()
if(VC_INSTALL_LIBS)
    set(install_components ${install_components} Libraries)
endif()
if(VC_INSTALL_UTILS)
    set(install_components ${install_components} Utilities)
endif()
if(VC_INSTALL_EXAMPLES)
    set(install_components ${install_components} Examples)
endif()
if(VC_INSTALL_DOC)
    set(install_components ${install_components} Documentation)
endif()

# Install resources (e.g. README, LICENSE, etc.) if anything else is
# getting installed
if(install_components)
install(
  FILES "doc/VC-Workflow.txt" "LICENSE"
  DESTINATION "${share_install_dir}"
  COMPONENT Resources
)
endif()

# Configure Cpack
set(CPACK_COMPONENTS_ALL ${install_components})
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_VENDOR "DRI")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Volume Cartographer")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PROJECT_VERSION}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")

if(APPLE)
    set(CPACK_GENERATOR TGZ DragNDrop)
    set(CPACK_DMG_FORMAT "UDBZ")
    set(CPACK_DMG_VOLUME_NAME "${PROJECT_NAME} ${PROJECT_VERSION}")
else()
    set(CPACK_GENERATOR TGZ ZIP)
endif()

include(CPack)
