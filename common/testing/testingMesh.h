//
// Created by Seth Parker on 9/18/15.
//

#ifndef VC_TESTINGMESH_H
#define VC_TESTINGMESH_H

#include "../vc_defines.h"
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>


namespace volcart {
namespace testing {

    // To-Do: Make this a base class so that the constructor can easily be reimplemented for different shapes
    class testingMesh {
    public:
        testingMesh();

        VC_MeshType::Pointer itkMesh();
        vtkPolyData* vtkMesh();

        //overload
        std::vector<VC_Vertex> getPoints() {return _points;}
        std::vector<VC_Cell> getCells() {return _cells;}

        //pcl exporter

    private:
        std::vector<VC_Vertex> _points;
        std::vector<VC_Cell> _cells;

        void _add_vertex(double x, double y, double z);
        void _add_cell(int v1, int v2, int v3);
        void _update_normal(int vertex, double nx_in, double ny_in, double nz_in);
    };

} // namespace testing
} // namespace volcart

#endif //VC_TESTINGMESH_H
