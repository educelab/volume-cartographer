# The prefix under which includes will be installed
set(vc_prefix "vc")

# Header install directory. Lowercase so includes look like:
#
#     #include <vc/core/types/PerPixelMap.h>
#
set(include_install_dir "include/${vc_prefix}")

# CMake config files
set(config_install_dir "lib/cmake/${PROJECT_NAME}")

# Extra resources
set(share_install_dir "share/${PROJECT_NAME}")

# Targets export name (VCConfig)
set(targets_export_name "${PROJECT_NAME}Targets")
set(namespace "${PROJECT_NAME}::")
