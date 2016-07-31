//
// Created by Seth Parker on 9/18/15.
//

#ifndef VC_TESTINGMESH_H
#define VC_TESTINGMESH_H

#include "vc_defines.h"
#include <pcl/common/common.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_types.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkSmartPointer.h>

namespace volcart
{
namespace shapes
{

// To-Do: Make this a base class so that the constructor can easily be
// reimplemented for different shapes
class ShapePrimitive
{
public:
    VC_MeshType::Pointer itkMesh();
    vtkSmartPointer<vtkPolyData> vtkMesh();
    pcl::PointCloud<pcl::PointXYZ> pointCloudXYZ(
        bool noisify = true);  // resamplePointCloud
    pcl::PointCloud<pcl::PointNormal>
    pointCloudNormal();  // poissonRecon, greedyProjMeshing
    pcl::PointCloud<pcl::PointXYZRGB>
    pointCloudXYZRGB();  // for orderedPCDMesher

    // overload
    std::vector<VC_Vertex> getPoints() { return _points; }
    std::vector<VC_Cell> getCells() { return _cells; }

protected:
    std::vector<VC_Vertex> _points;
    std::vector<VC_Cell> _cells;

    void _add_vertex(double x, double y, double z);
    void _add_cell(int v1, int v2, int v3);
    void _update_normal(int vertex, double nx_in, double ny_in, double nz_in);

    bool _orderedPoints;
    uint32_t _orderedWidth, _orderedHeight;
};

}  // namespace shapes
}  // namespace volcart

#endif  // VC_TESTINGMESH_H
