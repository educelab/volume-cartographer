//
// Created by Media Team on 7/7/15.
//

#ifndef VC_DEFINES_H
#define VC_DEFINES_H

// VC - System
#include <map>

// VC - ITK Mesh Defaults
#include <itkMesh.h>
#include "itkPointsLocator.h"
#include <itkTriangleCell.h>

// VC - OpenCV
#include <opencv2/opencv.hpp>

typedef itk::Vector< double, 3 >                      VC_PixelType;
typedef itk::Mesh< VC_PixelType, 3 >                  VC_MeshType;
typedef VC_MeshType::PointType                        VC_PointType;
typedef VC_MeshType::CellType                         VC_CellType;
typedef itk::TriangleCell< VC_CellType >              VC_TriangleType;

typedef VC_MeshType::PointsContainer                  VC_PointsContainerType;
typedef itk::PointsLocator<VC_PointsContainerType>    VC_PointsLocatorType;

typedef VC_MeshType::PointsContainer::ConstIterator   VC_PointsInMeshIterator;
typedef VC_MeshType::CellsContainer::Iterator         VC_CellIterator;
typedef VC_CellType::PointIdIterator                  VC_PointsInCellIterator;

typedef std::map<double, cv::Vec2d>                   VC_UVMap;

#endif //VC_DEFINES_H