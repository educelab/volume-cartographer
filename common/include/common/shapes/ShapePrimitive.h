//
// Created by Seth Parker on 9/18/15.
//
#pragma once

#include "../vc_defines.h"
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataReader.h>
#include <common/types/OrderedPointSet.h>
#include <common/types/Point.h>
#include <common/types/PointSet.h>


namespace volcart {
namespace shapes {

    class ShapePrimitive {
    public:
        VC_MeshType::Pointer itkMesh();
        vtkSmartPointer<vtkPolyData> vtkMesh();
        OrderedPointSet<Point3d> orderedPoints(bool noisify = false);
        PointSet<Point3d> unOrderedPoints(bool noisify = false);
        OrderedPointSet<Point6d> orderedPointNormal();
        PointSet<Point6d> unOrderedPointNormal();

        //overload
        std::vector<VC_Vertex> getPoints() const {return _points;}
        std::vector<VC_Cell> getCells() const {return _cells;}

        //ordering
        bool     isOrdered()     const { return _orderedPoints; };
        uint32_t orderedWidth()  const { return _orderedWidth;  };
        uint32_t orderedHeight() const { return _orderedHeight; };

    protected:
        std::vector<VC_Vertex> _points;
        std::vector<VC_Cell> _cells;

        void _add_vertex(double x, double y, double z);
        void _add_cell(int v1, int v2, int v3);
        void _update_normal(int vertex, double nx_in, double ny_in, double nz_in);

        bool _orderedPoints;
        uint32_t _orderedWidth, _orderedHeight;
    };

} // namespace shapes
} // namespace volcart
