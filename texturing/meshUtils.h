// meshUtils.h
// Abigail Coleman June 2015

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <vector>

#include <itkMesh.h>
#include <itkTriangleCell.h>
#include "itkMapContainer.h"

#include <vtkPolyDataNormals.h>

#include "vc_defines.h"
#include "io/ply2itk.h"
#include "itk2vtk.h"

#ifndef VC_MESHUTILS_H
#define VC_MESHUTILS_H

VC_MeshType::Pointer smoothNormals ( VC_MeshType::Pointer  inputMesh,
                                     double                smoothingFactor );

#endif // VC_MESHUTILS_H
