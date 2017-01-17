//
// Created by Seth Parker on 9/18/15.
//
#pragma once

#include <opencv2/core.hpp>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkSmartPointer.h>

#include "core/types/OrderedPointSet.hpp"
#include "core/types/PointSet.hpp"
#include "core/vc_defines.hpp"

namespace volcart
{
namespace shapes
{

class ShapePrimitive
{
public:
    ITKMesh::Pointer itkMesh();
    vtkSmartPointer<vtkPolyData> vtkMesh();
    OrderedPointSet<cv::Vec3d> orderedPoints(bool noisify = false);
    PointSet<cv::Vec3d> unorderedPoints(bool noisify = false);
    OrderedPointSet<cv::Vec6d> orderedPointNormal();
    PointSet<cv::Vec6d> unOrderedPointNormal();

    // overload
    std::vector<Vertex> getPoints() const { return points_; }
    std::vector<Cell> getCells() const { return cells_; }

    // ordering
    bool isOrdered() const { return orderedPoints_; }
    uint32_t orderedWidth() const { return orderedWidth_; }
    uint32_t orderedHeight() const { return orderedHeight_; }

protected:
    std::vector<Vertex> points_;
    std::vector<Cell> cells_;

    void addVertex_(double x, double y, double z);
    void addCell_(int v1, int v2, int v3);
    void updateNormal_(int vertex, double nx, double ny, double nz);

    bool orderedPoints_;
    uint32_t orderedWidth_, orderedHeight_;
};
}
}
