#pragma once

/** @file ITKMesh.hpp @ingroup Types */

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

/**
 * @brief Create exact copy of ITKMesh.
 *
 * Copy vertex and face information from the input mesh into the output mesh.
 * The resulting mesh is a unique (i.e. memory independent) copy of the
 * original.
 *
 * @warning Output parameter should point to an empty ITKMesh. This function
 * does not initialize a new ITKMesh::Pointer.
 *
 * @param copyVertices If `true,` copy vertices into the target mesh. Default:
 * `true`
 * @param copyFaces If `true,` copy faces into the target mesh. Default: `true`
 *
 * @ingroup Meshing
 */
void DeepCopy(
    const ITKMesh::Pointer& input,
    const ITKMesh::Pointer& output,
    bool copyVertices = true,
    bool copyFaces = true);

/**
 * @brief Create exact copy of ITKMesh.
 *
 * Copy vertex and face information from the input mesh into the output mesh.
 * The resulting mesh is a unique (i.e. memory independent) copy of the
 * original.
 */
auto DeepCopy(const ITKMesh::Pointer& input,bool copyVertices = true, bool copyFaces = true) -> ITKMesh::Pointer;
}  // namespace volcart
