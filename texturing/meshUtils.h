// meshUtils.h
// Abigail Coleman June 2015

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <vector>

#include <itkMesh.h>
#include <itkTriangleCell.h>
#include "itkMapContainer.h"
#include <itkMeshFileWriter.h>
#include <itkSTLMeshIOFactory.h>
#include <itkSTLMeshIO.h>

#include "vtkIsotropicDiscreteRemeshing.h"

#include "vc_defines.h"
#include "io/ply2itk.h"

#ifndef VC_MESHUTILS_H
#define VC_MESHUTILS_H

VC_MeshType::Pointer smoothNormals ( VC_MeshType::Pointer  inputMesh,
                                     double                smoothingFactor );

VC_MeshType::Pointer resample ( VC_MeshType::Pointer inputMesh );

#endif // VC_MESHUTILS_H
