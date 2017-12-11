# vc_addScale
add_executable(vc_addScale src/AddScale.cpp)
target_link_libraries(vc_addScale VC::core)
set(utils_install_list ${utils_install_list} vc_addScale)

# vc_area
add_executable(vc_area src/SurfArea.cpp)
target_link_libraries(vc_area VC::core VC::meshing Boost::filesystem)
set(utils_install_list ${utils_install_list} vc_area)

# vc_invertcloud
add_executable(vc_invertcloud src/InvertCloud.cpp)
target_link_libraries(vc_invertcloud VC::core Boost::filesystem)

# vc_mappingExtractor
add_executable(vc_mappingExtractor src/MappingExtractor.cpp)
target_link_libraries(vc_mappingExtractor VC::core)

# vc_csv2uv
add_executable(vc_csv2uv src/CSV2UV.cpp)
target_link_libraries(vc_csv2uv VC::core VC::meshing vtkIOPLY)

# vc_volpkg_v3to4
add_executable(vc_volpkg_v3to4 src/Volpkg3to4.cpp)
target_link_libraries(vc_volpkg_v3to4 VC::core)
set(utils_install_list ${utils_install_list} vc_volpkg_v3to4)

# vc_volpkg_v4to5
add_executable(vc_volpkg_v4to5 src/Volpkg4to5.cpp)
target_link_libraries(vc_volpkg_v4to5 VC::core)
set(utils_install_list ${utils_install_list} vc_volpkg_v4to5)

# Install targets
if(VC_INSTALL_UTILS)
install(
    TARGETS ${utils_install_list}
    RUNTIME DESTINATION bin
    COMPONENT Utilities
)
endif()
