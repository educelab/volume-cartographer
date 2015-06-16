// meshUtils.h
// Abigail Coleman June 2015

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <vector>

#include <itkMesh.h>
#include <itkTriangleCell.h>
#include "itkPointsLocator.h"
#include "itkMapContainer.h"

#ifndef VC_MESHUTILS_H
#define VC_MESHUTILS_H

itk::Mesh<::itk::Vector<double, 3>, 3>::Pointer smoothNormals ( itk::Mesh<::itk::Vector<double, 3>, 3>::Pointer inputMesh,
                                                                double smoothingFactor );

#endif // VC_MESHUTILS_H
