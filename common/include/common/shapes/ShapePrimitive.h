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
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>


namespace volcart {
namespace shapes {

    class ShapePrimitive {
    public:
        VC_MeshType::Pointer itkMesh();
        vtkSmartPointer<vtkPolyData> vtkMesh();
        pcl::PointCloud<pcl::PointXYZ> pointCloudXYZ(bool noisify = true); //resamplePointCloud
        pcl::PointCloud<pcl::PointNormal> pointCloudNormal(); //poissonRecon, greedyProjMeshing
        pcl::PointCloud<pcl::PointXYZRGB> pointCloudXYZRGB(); //for orderedPCDMesher

        //overload
        std::vector<VC_Vertex> getPoints() {return _points;}
        std::vector<VC_Cell> getCells() {return _cells;}

        //ordering
        bool     isOrdered()     { return _orderedPoints; };
        uint32_t orderedWidth()  { return _orderedWidth;  };
        uint32_t orderedHeight() { return _orderedHeight; };

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
