//
// Created by Media Team on 7/7/15.
//
#pragma once

#include <ctime>
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
#define VOLPKG_VERSION 2

// VC standard component access positions for XYZ
#define VC_INDEX_X 0
#define VC_INDEX_Y 1
#define VC_INDEX_Z 2

namespace volcart
{

// VC - Useful typedefs for general purpose use
typedef struct {
    double x, y, z, nx, ny, nz, s, t;
    int r, g, b, face_count;
} Vertex;

class Cell
{
public:
    Cell(){};
    Cell(unsigned long p1, unsigned long p2, unsigned long p3)
        : v1(p1), v2(p2), v3(p3){};
    unsigned long v1, v2, v3;
};

///// VC - ITK Mesh Defaults /////

typedef itk::Vector<double, 3> PixelType;
typedef itk::DefaultStaticMeshTraits<PixelType, 3, 3, double, double, PixelType>
    MeshTraits;
typedef itk::Mesh<PixelType, 3, MeshTraits> MeshType;
typedef MeshType::PointType PointType;
typedef MeshType::CellType CellType;
typedef itk::TriangleCell<CellType> TriangleType;

typedef MeshType::PointsContainer PointsContainerType;
typedef itk::PointsLocator<PointsContainerType> PointsLocatorType;

typedef MeshType::PointsContainer::ConstIterator PointsInMeshIterator;
typedef MeshType::CellsContainer::Iterator CellIterator;
typedef CellType::PointIdIterator PointsInCellIterator;

///// VC - ITK QuadEdgeMesh Defines /////

typedef double QuadPixel[3];
typedef itk::Vector<double, 3> QuadVector;
typedef itk::QuadEdgeMeshExtendedTraits<
    QuadVector,
    3,
    2,
    double,
    double,
    QuadVector,
    bool,
    bool>
    QuadTraits;
typedef itk::QuadEdgeMesh<double, 3, QuadTraits> QuadMesh;
typedef QuadMesh::PointType QuadPoint;
typedef QuadMesh::PointIdentifier QuadPointIdentifier;
typedef QuadMesh::CellType QuadCell;
typedef QuadMesh::CellIdentifier QuadCellIdentifier;
typedef itk::TriangleCell<QuadCell> QuadTriangleCell;
typedef QuadMesh::PointsContainer QuadPointsContainer;
typedef QuadMesh::PointsContainer::ConstIterator QuadPointsInMeshIterator;
typedef QuadMesh::CellsContainer::Iterator QuadCellIterator;
typedef QuadCell::PointIdIterator QuadPointsInCellIterator;
typedef QuadMesh::QEType QuadMeshQE;
typedef QuadMeshQE::IteratorGeom QuadMeshIteratorGeom;
typedef QuadMesh::EdgeListType QuadEdgeList;
typedef QuadMesh::EdgeListPointerType QuadEdgeListPointer;
typedef QuadEdgeList::iterator QuadEdgeListIterator;

///// ERROR MESSAGES /////
#define VC_ERR_READONLY()                                                 \
    {                                                                     \
        std::cerr << "ERROR: Volume Package is set to Read-Only. Cannot " \
                     "write to file."                                     \
                  << std::endl;                                           \
        return EXIT_FAILURE;                                              \
    }
#define VC_ERR_SLICE_ANALYZE                                               \
    "ERROR: Slice file does not exist/isn't a regular file and cannot be " \
    "analyze."

///// UV Maps /////
using Origin = cv::Vec2d;
using Voxel = cv::Vec3d;

#define VC_ORIGIN_TOP_LEFT volcart::Origin(0, 0)
#define VC_ORIGIN_TOP_RIGHT volcart::Origin(1, 0)
#define VC_ORIGIN_BOTTOM_LEFT volcart::Origin(0, 1)
#define VC_ORIGIN_BOTTOM_RIGHT volcart::Origin(1, 1)

#define VC_UVMAP_NULL_MAPPING cv::Vec2d(-1, -1)

struct Ratio {
    double width, height, aspect;
};

///// Texture Compositing /////

#define VC_TEXTURE_NO_VALUE -1.0

enum CompositeOption {
    Intersection = 0,
    NonMaximumSuppression,
    Maximum,
    Minimum,
    MedianAverage,
    Median,
    Mean
};

enum DirectionOption { Bidirectional = 0, Positive, Negative };

///// Time Helper /////

inline std::string DATE_TIME()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *std::localtime(&now);
    std::strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", &tstruct);
    return std::string(buf);
}
}  // namespace volcart
