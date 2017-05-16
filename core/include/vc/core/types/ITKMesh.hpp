/** @file ITKMesh.hpp @ingroup Types */
#pragma once

#include <itkMesh.h>
#include <itkPointsLocator.h>
#include <itkTriangleCell.h>

namespace volcart
{

/** @name ITK Mesh */
/**@{*/
/** ITK Vertex/Pixel base properties */
using ITKPixel = itk::Vector<double, 3>;

/** ITK Mesh base properties */
using ITKMeshTraits =
    itk::DefaultStaticMeshTraits<ITKPixel, 3, 3, double, double, ITKPixel>;

/** ITK Mesh */
using ITKMesh = itk::Mesh<ITKPixel, 3, ITKMeshTraits>;

/** ITK Mesh Vertex */
using ITKPoint = ITKMesh::PointType;

/** ITK Mesh Generic Face */
using ITKCell = ITKMesh::CellType;

/** ITK Mesh Triangular Face */
using ITKTriangle = itk::TriangleCell<ITKCell>;

/** ITK Mesh Point Container */
using ITKPointsContainer = ITKMesh::PointsContainer;

/** ITK Mesh Point Locator */
using ITKPointsLocator = itk::PointsLocator<ITKPointsContainer>;

/** ITK Mesh Point Iterator */
using ITKPointIterator = ITKPointsContainer::ConstIterator;

/** ITK Mesh Cell Iterator */
using ITKCellIterator = ITKMesh::CellsContainer::Iterator;

/** ITK Mesh Vertex ID In Cell Iterator */
using ITKPointInCellIterator = ITKCell::PointIdIterator;
/**@}*/
}
