# Install resources (e.g. README, LICENSE, etc.)
install(
  FILES 
    "${PROJECT_SOURCE_DIR}/doc/VC-Workflow.txt"
    "${PROJECT_SOURCE_DIR}/LICENSE"
  DESTINATION .
  COMPONENT Resources
)

# What components to install
set(INSTALL_COMPONENTS "Libraries" "Includes" "Resources")
if(VC_INSTALL_APPS)
    list(APPEND INSTALL_COMPONENTS "Programs")
endif()
if(VC_INSTALL_UTILS)
    list(APPEND INSTALL_COMPONENTS "Utilities")
endif()
if(VC_INSTALL_EXAMPLES)
    list(APPEND INSTALL_COMPONENTS "Examples")
endif()

# Configure Cpack
set(CPACK_GENERATOR "TGZ;ZIP")
set(CPACK_COMPONENTS_ALL ${INSTALL_COMPONENTS})
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_VENDOR "UK VisCenter")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Volume Cartographer")

if(APPLE)
     set(CPACK_GENERATOR "DragNDrop")
     set(CPACK_DMG_FORMAT "UDBZ")
     set(CPACK_DMG_VOLUME_NAME "${PROJECT_NAME}")
     set(CPACK_SYSTEM_NAME "OSX")
     set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PROJECT_VERSION}")
elseif(UNIX)
    set(CPACK_SYSTEM_NAME "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
endif()

include(CPack)
