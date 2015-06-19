###############################################################################
# Find ACVD
#
# This sets the following variables:
# ACVD_FOUND - True if ACVD was found.
# ACVD_INCLUDE_DIRS - Directories containing the ACVD include files.
# ACVD_LIBRARIES - Libraries needed to use ACVD.
# ACVD_DEFINITIONS - Compiler flags for ACVD.

#add a hint so that it can find it without the pkg-config
find_path ( ACVD_INCLUDE_DIR XnStatus.h
            HINTS ${NESTK_ROOT_DIRS_HINTS} ${PC_ACVD_INCLUDEDIR} ${PC_ACVD_INCLUDE_DIRS} /usr/include/ACVD /usr/local/include/ACVD
            PATHS "$ENV{PROGRAMFILES}/ACVD/Include" "$ENV{PROGRAMW6432}/ACVD/Include"
            PATH_SUFFIXES acvd ACVD )
#add a hint so that it can find it without the pkg-config
find_library ( ACVD_LIBRARY
                NAMES ACVD64 ACVD
                HINTS ${NESTK_ROOT_DIRS_HINTS} ${PC_ACVD_LIBDIR} ${PC_ACVD_LIBRARY_DIRS} /usr/lib /usr/local/lib
                PATHS "$ENV{PROGRAMFILES}/ACVD/Lib${ACVD_SUFFIX}" "$ENV{PROGRAMW6432}/ACVD/Lib${ACVD_SUFFIX}" "$ENV{PROGRAMW6432}/ACVD"
                PATH_SUFFIXES lib lib64 )

include ( FindPackageHandleStandardArgs )
find_package_handle_standard_args ( ACVD DEFAULT_MSG
    ACVD_LIBRARY ACVD_INCLUDE_DIR )

mark_as_advanced ( ACVD_LIBRARY ACVD_INCLUDE_DIR )
if( ACVD_FOUND )
    include_directories ( ${ACVD_INCLUDE_DIRS} )
    message ( STATUS "ACVD found (include: ${ACVD_INCLUDE_DIR}, lib: ${ACVD_LIBRARY})" )
endif ( ACVD_FOUND )
