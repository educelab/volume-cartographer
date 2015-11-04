###############################################################################
# Find ACVD Remeshing Library

find_path(ACVD_COMMON_INCLUDE_DIR vtkSurface.h
          HINTS /usr/include/ /usr/local/include/
          PATH_SUFFIXES ACVD/ ACVD/Common)

find_path(ACVD_REMESHING_INCLUDE_DIR vtkDiscreteRemeshing.h
          HINTS /usr/include/ /usr/local/include/
          PATH_SUFFIXES ACVD/ ACVD/DiscreteRemeshing)

find_path(ACVD_PROCESSING_INCLUDE_DIR vtkImageDataCleanLabels.h
          HINTS /usr/include/ /usr/local/include/
          PATH_SUFFIXES ACVD/ ACVD/VolumeProcessing)

find_library(ACVD_LIBRARIES NAMES vtkSurface vtkDiscreteRemeshing vtkVolumeProcessing
          HINTS /usr/lib/ /usr/local/lib)

set(ACVD_INCLUDE_DIRS ${ACVD_COMMON_INCLUDE_DIR} ${ACVD_REMESHING_INCLUDE_DIR} ${ACVD_PROCESSING_INCLUDE_DIR})
