/** @file QuadEdgeMesh.hpp @ingroup Types */
#pragma once

#include <array>

#include <itkQuadEdgeMesh.h>
#include <itkQuadEdgeMeshExtendedTraits.h>
#include <itkTriangleCell.h>

namespace volcart
{

/** @name ITK QuadEdgeMesh */
/**@{*/
/** ITK QEM Vertex/Pixel base properties */
using QuadPixel = std::array<double, 3>;

/** ITK QEM Cell Pixel base properties */
using QuadVector = itk::Vector<double, 3>;

/** ITK QEM Mesh base properties */
using QuadTraits = itk::QuadEdgeMeshExtendedTraits<
    QuadVector,
    3,
    2,
    double,
    double,
    QuadVector,
    bool,
    bool>;

/** ITK Quad-Edge Mesh
 *
 * http://www.insight-journal.org/browse/publication/122
 */
using QuadEdgeMesh = itk::QuadEdgeMesh<double, 3, QuadTraits>;

/** ITK QEM Vertex */
using QuadPoint = QuadEdgeMesh::PointType;

/** ITK QEM Vertex ID */
using QuadPointIdentifier = QuadEdgeMesh::PointIdentifier;

/** ITK QEM Generic Face */
using QuadCell = QuadEdgeMesh::CellType;

/** ITK QEM Cell ID */
using QuadCellIdentifier = QuadEdgeMesh::CellIdentifier;

/** ITK QEM Triangular Face */
using QuadTriangleCell = itk::TriangleCell<QuadCell>;

/** ITK QEM Point Container */
using QuadPointsContainer = QuadEdgeMesh::PointsContainer;

/** ITK QEM Point Container Iterator */
using QuadPointsInMeshIterator = QuadEdgeMesh::PointsContainer::ConstIterator;

/** ITK QEM Face Iterator */
using QuadCellIterator = QuadEdgeMesh::CellsContainer::Iterator;

/** ITK QEM Vertex ID In Cell Iterator */
using QuadPointsInCellIterator = QuadCell::PointIdIterator;

/** ITK QEM Edge */
using QuadMeshQE = QuadEdgeMesh::QEType;

/** ITK QEM Geometry Iterator */
using QuadMeshIteratorGeom = QuadMeshQE::IteratorGeom;

/** ITK QEM Edge List */
using QuadEdgeList = QuadEdgeMesh::EdgeListType;

/** ITK QEM Edge List Pointer */
using QuadEdgeListPointer = QuadEdgeMesh::EdgeListPointerType;

/** ITK QEM Edge List Iterator */
using QuadEdgeListIterator = QuadEdgeList::iterator;
/**@}*/
}