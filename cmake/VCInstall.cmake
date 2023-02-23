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
if(VC_INSTALL_DOCS)
    set(install_components ${install_components} Documentation)
endif()

# Install resources (e.g. README, LICENSE, etc.) if anything else is
# getting installed
if(install_components)
install(
  FILES "LICENSE"
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
    set(CPACK_GENERATOR DragNDrop)
    set(CPACK_DMG_FORMAT "UDBZ")
    set(CPACK_DMG_VOLUME_NAME "${PROJECT_NAME} ${PROJECT_VERSION}")
else()
    set(CPACK_GENERATOR TGZ ZIP)
endif()

include(CPack)
