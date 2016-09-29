//
// Created by Media Team on 7/7/15.
//
#pragma once

#include <array>
#include <ctime>
#include <iostream>
#include <itkMesh.h>
#include <itkPointsLocator.h>
#include <itkQuadEdgeMesh.h>
#include <itkQuadEdgeMeshExtendedTraits.h>
#include <itkTriangleCell.h>
#include <opencv2/core/core.hpp>
#include "common/util/getMemorySize.h"

///// GLOBAL VARIABLES /////
// This define determines the default dictionary used when creating a new
// volumepkg
constexpr static int VOLPKG_VERSION = 2;

// VC standard component access positions for XYZ
constexpr static int VC_INDEX_X = 0;
constexpr static int VC_INDEX_Y = 1;
constexpr static int VC_INDEX_Z = 2;

// VC - Useful typedefs for general purpose use
namespace volcart
{

struct Vertex {
    double x, y, z, nx, ny, nz, s, t;
    int r, g, b, face_count;
};

struct Cell {
    uint64_t v1, v2, v3;
    Cell() : v1{}, v2{}, v3{} {}
    Cell(uint64_t p1, uint64_t p2, uint64_t p3) : v1{p1}, v2{p2}, v3{p3} {};
};

struct Ratio {
    double width, height, aspect;
};

///// VC - ITK Mesh Defaults /////
using PixelType = itk::Vector<double, 3>;
using MeshTraits =
    itk::DefaultStaticMeshTraits<PixelType, 3, 3, double, double, PixelType>;
using MeshType = itk::Mesh<PixelType, 3, MeshTraits>;
using PointType = MeshType::PointType;
using CellType = MeshType::CellType;
using TriangleType = itk::TriangleCell<CellType>;

using PointsContainerType = MeshType::PointsContainer;
using PointsLocatorType = itk::PointsLocator<PointsContainerType>;

using PointsInMeshIterator = MeshType::PointsContainer::ConstIterator;
using CellIterator = MeshType::CellsContainer::Iterator;
using PointsInCellIterator = CellType::PointIdIterator;

///// VC - ITK QuadEdgeMesh Defines /////
using QuadPixel = std::array<double, 3>;
using QuadVector = itk::Vector<double, 3>;
using QuadTraits = itk::QuadEdgeMeshExtendedTraits<
    QuadVector,
    3,
    2,
    double,
    double,
    QuadVector,
    bool,
    bool>;
using QuadMesh = itk::QuadEdgeMesh<double, 3, QuadTraits>;
using QuadPoint = QuadMesh::PointType;
using QuadPointIdentifier = QuadMesh::PointIdentifier;
using QuadCell = QuadMesh::CellType;
using QuadCellIdentifier = QuadMesh::CellIdentifier;
using QuadTriangleCell = itk::TriangleCell<QuadCell>;
using QuadPointsContainer = QuadMesh::PointsContainer;
using QuadPointsInMeshIterator = QuadMesh::PointsContainer::ConstIterator;
using QuadCellIterator = QuadMesh::CellsContainer::Iterator;
using QuadPointsInCellIterator = QuadCell::PointIdIterator;
using QuadMeshQE = QuadMesh::QEType;
using QuadMeshIteratorGeom = QuadMeshQE::IteratorGeom;
using QuadEdgeList = QuadMesh::EdgeListType;
using QuadEdgeListPointer = QuadMesh::EdgeListPointerType;
using QuadEdgeListIterator = QuadEdgeList::iterator;

///// ERROR MESSAGES /////
inline int VC_ERR_READONLY()
{
    std::cerr << "ERROR: VolPkg is set to read-only. Cannot write to file"
              << std::endl;
    return EXIT_FAILURE;
}

///// UV Maps /////
using Origin = cv::Vec2d;
using Voxel = cv::Vec3d;

#define VC_ORIGIN_TOP_LEFT VC_Origin(0, 0)
#define VC_ORIGIN_TOP_RIGHT VC_Origin(1, 0)
#define VC_ORIGIN_BOTTOM_LEFT VC_Origin(0, 1)
#define VC_ORIGIN_BOTTOM_RIGHT VC_Origin(1, 1)

#define VC_UVMAP_NULL_MAPPING cv::Vec2d(-1, -1)

///// Texture Compositing /////
constexpr static double VC_TEXTURE_NO_VALUE = -1.0;
enum class CompositeOption {
    Intersection,
    NonMaximumSuppression,
    Maximum,
    Minimum,
    MedianAverage,
    Median,
    Mean
};

enum class DirectionOption { Bidirectional, Positive, Negative };

///// Time Helper /////
inline std::string DATE_TIME()
{
    time_t now = std::time(0);
    struct tm tstruct;
    std::array<char, 80> buf;
    tstruct = *std::localtime(&now);
    std::strftime(buf.data(), buf.size(), "%Y%m%d%H%M%S", &tstruct);
    return std::string(buf.data());
}
}  // namespace volcart
