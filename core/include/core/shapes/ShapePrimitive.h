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

#include "core/types/OrderedPointSet.h"
#include "core/types/PointSet.h"
#include "core/vc_defines.h"

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
    std::vector<Vertex> getPoints() const { return _points; }
    std::vector<Cell> getCells() const { return _cells; }

    // ordering
    bool isOrdered() const { return _orderedPoints; }
    uint32_t orderedWidth() const { return _orderedWidth; }
    uint32_t orderedHeight() const { return _orderedHeight; }

protected:
    std::vector<Vertex> _points;
    std::vector<Cell> _cells;

    void _add_vertex(double x, double y, double z);
    void _add_cell(int v1, int v2, int v3);
    void _update_normal(int vertex, double nx_in, double ny_in, double nz_in);

    bool _orderedPoints;
    uint32_t _orderedWidth, _orderedHeight;
};
}  // namespace shapes
}  // namespace volcart
